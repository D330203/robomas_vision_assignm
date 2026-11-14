//! Predict the moving target and return an aiming command in [`aim`].
//! World axes: +X forward, +Y left, +Z up. Distances are metres and angles are radians.
//! The simulator calls `aim` every 10 ms with a perfect observation captured in the past.

pub mod sim;
pub mod ui;

/// Fixed launcher physics and timing shared by the controller, viewer, and evaluator.
pub mod physics {
    /// Stationary muzzle position in world metres.
    pub const MUZZLE_M: [f64; 3] = [0.0, 0.0, 0.3];
    /// Projectile launch speed in metres per second.
    pub const MUZZLE_SPEED_M_S: f64 = 25.0;
    /// Downward acceleration in metres per second squared.
    pub const GRAVITY_M_S2: f64 = 9.81;
    /// Time from observation capture to delivery.
    pub const OBSERVATION_DELAY_S: f64 = 0.05;
    /// Time from command acceptance to projectile launch.
    pub const LAUNCH_DELAY_S: f64 = 0.02;
    /// Minimum interval between accepted shots.
    pub const FIRE_PERIOD_S: f64 = 0.25;
    /// Projectile lifetime after launch.
    pub const MAX_FLIGHT_S: f64 = 2.0;
    /// Fixed interval between controller calls.
    pub const CONTROL_PERIOD_S: f64 = 0.01;
}

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use std::f64::consts::TAU;
use ui::{Color, Gizmos};

/// 调用核心功能函数aim（）求解弹道，input只读输入，mutGizmos允许函数添加调试信息
///返回值为Option<Command>，如果没有观测数据则返回None
pub fn aim(input: &Input, debug: &mut Gizmos) -> Option<Command> {
    let resolved = input.stats.hits + input.stats.misses;
    debug.overlay("Hit rate", format!("{}/{resolved}", input.stats.hits));

    let observation = input.observation?;
    /// 观测年龄 = 现在时间 - 观测时间
    let age_s = input.now_s - observation.time_s; 
    if !age_s.is_finite() || age_s < 0.0 {
        return None;
    }
    for plate in observation.target.plates(input.size) {
        debug.plate(plate, input.size, Color::Cyan);
    }

    let until_launch_s = age_s + physics::LAUNCH_DELAY_S;
    let target_at_launch = observation.target.at(until_launch_s);
    /// 确定中心点位置
    let center_m = [
        target_at_launch.x_m,
        target_at_launch.y_m,
        input.size.plate_center_z_m,
    ];
    let offset_m = subtract(center_m, physics::MUZZLE_M);
    let rough_flight_s = dot(offset_m, offset_m).sqrt() / physics::MUZZLE_SPEED_M_S;

    // 计算目标板的相对速度和入射角度
    let rough_plates = observation
        .target
        .at(until_launch_s + rough_flight_s)
        .plates(input.size);
    let (slot, _) = rough_plates
        .iter()
        .enumerate()
        .filter_map(|(slot, plate)| {
            let to_muzzle = subtract(physics::MUZZLE_M, plate.center_m);
            let facing = normal_alignment(plate.outward_normal, to_muzzle)?;
            (facing > 0.0).then_some((slot, facing))
        })
        .max_by(|a, b| a.1.total_cmp(&b.1))?;

    let (trajectory, plate) = solve_intercept(
        observation.target,
        input.size,
        slot,
        until_launch_s,
        rough_flight_s,
    )?;
    ///相对速度 = 目标速度 - 弹丸速度
    let relative_velocity = subtract(
        trajectory.velocity(trajectory.flight_time_s),
        plate.velocity_m_s,
    );
    let incidence = -normal_alignment(plate.outward_normal, relative_velocity)?;
    // 入射评分：先设定常量：子弹抛物运动向量减去目标板的速度向量，得到相对速度向量，
    // 然后计算相对速度向量与板法线的夹角余弦值（大于零）
    // 余弦值越大，说明子弹入射角度越接近板法线方向，命中概率越高
    // 余弦最小值设为0.25，低于该值说明夹角过大，系统不开火
    const MIN_INCIDENCE_COS: f64 = 0.25;
    //设定模拟渲染颜色
    debug.plate(plate, input.size, Color::Green);
    debug.trajectory(trajectory, Color::Orange);
    debug.point(
        "Impact",
        trajectory.position(trajectory.flight_time_s),
        Color::Green,
    );
    debug.overlay(
        "Flight",
        format!("{:.1} ms", 1000.0 * trajectory.flight_time_s),
    );
    debug.overlay("Incidence", format!("{incidence:.2}"));

    Some(Command {
        yaw_rad: trajectory.yaw_rad,
        pitch_rad: trajectory.pitch_rad,
        fire: incidence >= MIN_INCIDENCE_COS,
    })
}

fn solve_intercept(
    target: Target,
    size: Size,
    slot: usize,
    until_launch_s: f64,
    mut flight_s: f64,
) -> Option<(Trajectory, Plate)> {
//瞄准的装甲板编号只在单次迭代固定，重新迭代时，目标板可以重新选择
    const TIME_TOLERANCE_S: f64 = 1e-7;
    const MAX_ITERATIONS: usize = 32;
// 迭代求解到目标板的飞行时间，122行设定最大迭代次数32次，121行设定两次迭代误差允许范围为1e-7秒
// 由于开始迭代时，估算的飞行时间没有考虑目标版移动的因素，故需要每次加入目标版在该段时间内移动的距离
// 重新计算飞行时间，直到两次计算的飞行时间差小于误差允许范围
// 考虑到某些情况（例如目标版加速远离，迭代无法收敛）设定最大迭代次数，避免陷入死循环，节省计算资源。
    for _ in 0..MAX_ITERATIONS {
    /// 迭代起点，总时间 = 发射延迟 + roughflight（飞行估算）   
        let plate = target.at(until_launch_s + flight_s).plates(size)[slot];
        let trajectory = Trajectory::solve(plate.center_m)?;
        let next_flight_s = trajectory.flight_time_s;
        if (next_flight_s - flight_s).abs() <= TIME_TOLERANCE_S {
            // 求解出的飞行时间同时用于检测击中和模拟绘制
            let impact_plate = target.at(until_launch_s + next_flight_s).plates(size)[slot];
            return Some((trajectory, impact_plate));
        }
        flight_s = next_flight_s;
    }
    None
}

fn subtract(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    std::array::from_fn(|i| a[i] - b[i])
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a.into_iter().zip(b).map(|(x, y)| x * y).sum()
}

fn normal_alignment(unit_normal: [f64; 3], direction: [f64; 3]) -> Option<f64> {
    let length = dot(direction, direction).sqrt();
    if !length.is_finite() || length <= 0.0 {
        return None;
    }
    let alignment = dot(unit_normal, direction) / length;
    alignment.is_finite().then_some(alignment)
}

/// 仿真器执行，检测输入
/// The only simulation data provided to the applicant's controller.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct Input {
    pub now_s: f64,
    /// None until the first observation's configured delivery delay has elapsed.
    pub observation: Option<Sample>,
    pub size: Size,
    /// Outcomes resolved by this tick, before the current command is accepted.
    #[serde(default)]
    pub stats: Stats,
}

/// 初始化幅角
/// World yaw, upward pitch, and a request to fire at the fixed maximum rate.
/// Angles are latched when a shot is accepted, then applied after `physics::LAUNCH_DELAY_S`.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct Command {
    pub yaw_rad: f64,
    pub pitch_rad: f64,
    pub fire: bool,
}

/// 模拟只考虑重力
/// A gravity-only projectile path. Flight time starts at launch, excluding all delays.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Trajectory {
    pub yaw_rad: f64,
    pub pitch_rad: f64,
    pub flight_time_s: f64,
}

impl Trajectory {
    /// 弹道求解，包含在迭代步骤中
    /// 根据目标距离（向量）反推飞行时间，解二次方程求得较小的飞行时间t（即较平的弹道）
    /// 用该飞行时间推算俯仰角，计算弹道
    /// 若求解出的飞行时间不合理（负数，复数或大于两秒）则返回None，不开火
    /// Solve the low arc to a fixed world point at the fixed muzzle speed.
    /// Returns `None` for an unreachable point, nonfinite input, zero flight time, or timeout.
    /// Predict the moving target separately; this method does not compensate for delays.
    pub fn solve(point_m: [f64; 3]) -> Option<Self> {
        let delta: [f64; 3] = std::array::from_fn(|i| point_m[i] - physics::MUZZLE_M[i]);
        let distance_squared = delta.iter().map(|v| v * v).sum::<f64>();
        let gravity = physics::GRAVITY_M_S2;
        let a = physics::MUZZLE_SPEED_M_S.powi(2) - gravity * delta[2];
        let discriminant = a * a - gravity * gravity * distance_squared;
        if a <= 0.0 || discriminant < 0.0 {
            return None;
        }
        // Rationalized smaller root of the ballistic equation in flight time squared.
        let flight_time_s = (2.0 * distance_squared / (a + discriminant.sqrt())).sqrt();
        if !flight_time_s.is_finite()
            || flight_time_s <= 0.0
            || flight_time_s > physics::MAX_FLIGHT_S
        {
            return None;
        }
        let mut velocity = delta.map(|d| d / flight_time_s);
        velocity[2] += 0.5 * gravity * flight_time_s;
        Some(Self {
            yaw_rad: velocity[1].atan2(velocity[0]),
            pitch_rad: velocity[2].atan2(velocity[0].hypot(velocity[1])),
            flight_time_s,
        })
    }

    /// World position `elapsed_s` after launch; gravity produces a mathematical parabola.
    pub fn position(self, elapsed_s: f64) -> [f64; 3] {
        let velocity = self.velocity(0.0);
        std::array::from_fn(|i| {
            physics::MUZZLE_M[i] + velocity[i] * elapsed_s
                - if i == 2 {
                    0.5 * physics::GRAVITY_M_S2 * elapsed_s * elapsed_s
                } else {
                    0.0
                }
        })
    }
    /// 相对于地面速度
    /// World velocity `elapsed_s` after launch, before subtracting the armor's velocity.
    pub fn velocity(self, elapsed_s: f64) -> [f64; 3] {
        let horizontal = physics::MUZZLE_SPEED_M_S * self.pitch_rad.cos();
        [
            horizontal * self.yaw_rad.cos(),
            horizontal * self.yaw_rad.sin(),
            physics::MUZZLE_SPEED_M_S * self.pitch_rad.sin() - physics::GRAVITY_M_S2 * elapsed_s,
        ]
    }
}

/// World axes are +X forward, +Y left, +Z up. Angles turn from +X toward +Y.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Target {
    pub x_m: f64,
    pub y_m: f64,
    pub theta_rad: f64,
    pub vx_m_s: f64,
    pub vy_m_s: f64,
    pub omega_rad_s: f64,
}

impl Target {
    /// Evaluate constant world velocity and spin without integrating intermediate steps.
    pub fn at(self, elapsed_s: f64) -> Self {
        Self {
            x_m: self.x_m + self.vx_m_s * elapsed_s,
            y_m: self.y_m + self.vy_m_s * elapsed_s,
            theta_rad: (self.theta_rad + self.omega_rad_s * elapsed_s).rem_euclid(TAU),
            ..self
        }
    }

    /// Four equally spaced plates, mounted with their top edges tilted toward the center.
    pub fn plates(self, size: Size) -> [Plate; 4] {
        std::array::from_fn(|slot| {
            let angle = self.theta_rad + slot as f64 * TAU / 4.0;
            let (sin, cos) = angle.sin_cos();
            let (tilt_sin, tilt_cos) = size.mount_pitch_rad.sin_cos();
            Plate {
                center_m: [
                    self.x_m + size.radius_m * cos,
                    self.y_m + size.radius_m * sin,
                    size.plate_center_z_m,
                ],
                outward_normal: [cos * tilt_cos, sin * tilt_cos, tilt_sin],
                right_axis: [-sin, cos, 0.0],
                up_axis: [-cos * tilt_sin, -sin * tilt_sin, tilt_cos],
                velocity_m_s: [
                    self.vx_m_s - self.omega_rad_s * size.radius_m * sin,
                    self.vy_m_s + self.omega_rad_s * size.radius_m * cos,
                    0.0,
                ],
            }
        })
    }
}

/// Scoring rectangle and mounting geometry. Plates have equal radii and heights.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(default, deny_unknown_fields)]
pub struct Size {
    /// Positive pitch tilts the outward normal up, matching the main simulator.
    pub mount_pitch_rad: f64,
    pub radius_m: f64,
    pub plate_width_m: f64,
    pub plate_height_m: f64,
    pub plate_center_z_m: f64,
}

/// 
/// Ground-truth plate state; plate indices remain attached to the target as it turns.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Plate {
    pub center_m: [f64; 3],
    pub outward_normal: [f64; 3],
    pub right_axis: [f64; 3],
    pub up_axis: [f64; 3],
    pub velocity_m_s: [f64; 3],
}

impl Plate {
    /// Convert plate coordinates (right, up, outward), in metres, to world coordinates.
    pub fn point(self, right_m: f64, up_m: f64, outward_m: f64) -> [f64; 3] {
        std::array::from_fn(|i| {
            self.center_m[i]
                + self.right_axis[i] * right_m
                + self.up_axis[i] * up_m
                + self.outward_normal[i] * outward_m
        })
    }
}

/// Perfect target state at the original observation capture time.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Sample {
    pub time_s: f64,
    pub target: Target,
}

/// Accepted shots and their outcomes. Requests blocked by the firing rate are not shots.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct Stats {
    pub shots: usize,
    pub hits: usize,
    pub misses: usize,
    pub invalid_commands: usize,
}
