所选路线：瞄准

依赖：
Rust 1.98.1 or higher (与作业要求同)
WGPU supporting environment (与作业要求同)
其余rust 依赖cargo会自动获取
ffmpeg用于导出.mp4

测试运行：
在aiming目录下终端输入：
cargo run --locked -- --config scenarios/moving.json
及
cargo run --locked -- --config scenarios/spinning.json
及
cargo run --locked -- --config scenarios/stationary.json

