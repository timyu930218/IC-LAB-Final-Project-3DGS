# IC-LAB-Final-Project-3DGS
This project implements an RTL-based hardware accelerator for 3D Gaussian Splatting (3DGS), targeting efficient real-time rendering through dedicated digital hardware. The design focuses on key stages of the 3DGS pipeline, including Gaussian preprocessing, projection, depth sorting, rasterization, and alpha blending.

The accelerator is implemented in Verilog with an emphasis on pipelining and parallelism. A hardware Bitonic Sort module is used for depth sorting, while computationally expensive operations such as square-root calculation are approximated using lookup tables to reduce latency and hardware complexity. The design targets a 5 ns clock period.

A software reference model is included for functional verification. Its outputs are compared with RTL simulation results to validate individual modules and the overall rendering pipeline.

The current prototype supports a resolution of 128 × 128 pixels and up to 64 Gaussian primitives. This repository contains the RTL implementation and software reference code developed for the IC Laboratory final project.

## License and Copyright

Copyright © 2026 Kai-An You. All Rights Reserved.

The source code and materials in this repository are proprietary and are made publicly available solely for viewing and evaluation purposes.

Permission is not granted to use, copy, modify, redistribute, publish, or incorporate any portion of this project into other academic, research, or commercial work.

Anyone wishing to obtain permission to use any part of this repository must contact the author and receive explicit written authorization in advance.

For permission requests, please contact: timyu930218@gmail.com
