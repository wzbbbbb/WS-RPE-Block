# WS-RPE-Block
WS-RPE Block: FPGA Architecture for Real-Time Eigenvalue Computation in Edge Control Systems Journal Submission: IEEE Transactions on Very Large Scale Integration (TVLSI) Systems


Overview
WS-RPE Block is an innovative hardware architecture designed for real-time eigenvalue computation in edge Physics-AI systems. The architecture implements dynamic scheduling mechanisms to efficiently handle the small, sparse, and evolving matrices typical of control system stability monitoring.

The core innovation lies in the co-design of work-stealing and redundant-PE activation as first-class hardware primitives, enabling adaptive computation that matches the dynamic nature of control system workloads.

 Architecture Overview
Core Components
The architecture consists of several key modules that work together to provide dynamic, adaptive computation:

Dynamic Work-Stealing Controller: Redistributes tasks from overloaded to idle pipelines

Redundant-PE Mapper: Activates secondary computation paths during congestion

Block Partitioner: Adaptively partitions matrices into optimal sub-blocks

Reorder Buffer: Manages out-of-order computation results

Dynamic Adjustment Unit: Adapts convergence parameters based on workload patterns

Key Features
Multi-Size Block Parallelism: Supports 2×2 to N×N block decomposition

Cross-Queue Task Redistribution: Hardware-implemented work stealing

On-Demand Redundancy: Dual-path execution for congested pipelines

Adaptive Convergence: Dynamic threshold adjustment using temporal correlation

Standard Interfaces: AXI-Stream for easy system integration

Project Structure
text
ws_rpe_project/
├── rtl/                              # RTL Source Code
│   ├── ws_rpe_top.sv                 # Top-level module with AXI-Stream interfaces
│   ├── ws_rpe_engine.sv              # Main computation engine
│   ├── work_stealing_controller.sv   # Dynamic task redistribution
│   ├── redundant_pe_mapper.sv        # Redundant PE activation
│   ├── processing_element.sv         # 2×1 PE core
│   ├── block_partitioner.sv          # Adaptive matrix partitioning
│   ├── reorder_buffer.sv             # Result reordering
│   ├── dynamic_adjustment_unit.sv    # Adaptive convergence control
│   ├── convergence_controller.sv     # Convergence checking
│   ├── block_scheduler.sv            # Task scheduling
│   ├── vector_combiner.sv            # Vector assembly
│   ├── performance_monitor.sv        # Performance metrics
│   ├── system_controller.sv          # System control
│   └── input_processor.sv            # Input processing
├── sim/                              # Simulation Environment
│   ├── tb_ws_rpe.sv                  # Top-level testbench
│   ├── test_cases/                   # Test matrices
│   └── verification/                 # Verification utilities
├── scripts/                          # Build Scripts
│   ├── synthesis.tcl                 # Vivado synthesis
│   ├── implementation.tcl            # Implementation
│   └── simulation.tcl                # Simulation
├── constraints/                      # FPGA Constraints
│   └── timing.xdc                    # Timing constraints
└── docs/                             # Documentation
    └── architecture.md               # Architecture specification
