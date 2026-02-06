module ws_rpe_engine #(
    parameter MATRIX_SIZE = 32,
    parameter DATA_WIDTH = 32,
    parameter NUM_PIPELINES = 16,
    parameter MAX_ITER = 100,
    parameter THRESHOLD = 1e-6
)(
    input logic clk,
    input logic rst_n,
    input logic [DATA_WIDTH-1:0] matrix_in [MATRIX_SIZE-1:0][MATRIX_SIZE-1:0],
    input logic [DATA_WIDTH-1:0] vector_in [MATRIX_SIZE-1:0],
    input logic matrix_valid,
    input logic [31:0] config,
    output logic [DATA_WIDTH-1:0] vector_out [MATRIX_SIZE-1:0],
    output logic [DATA_WIDTH-1:0] eigenvalue,
    output logic done,
    output logic [7:0] iter_count,
    output logic error
);

// 内部状态定义
typedef enum logic [2:0] {
    IDLE = 3'b000,
    INIT = 3'b001,
    BLOCK_PARTITION = 3'b010,
    SCHEDULE = 3'b011,
    COMPUTE = 3'b100,
    CONVERGE_CHECK = 3'b101,
    DONE_STATE = 3'b110
} state_t;

state_t current_state, next_state;

// 控制信号
logic start;
logic partition_done;
logic schedule_done;
logic compute_done;
logic converge;

// 任务队列相关
logic [DATA_WIDTH-1:0] task_queue [NUM_PIPELINES-1:0][$];
logic [7:0] queue_size [NUM_PIPELINES-1:0];
logic queue_empty [NUM_PIPELINES-1:0];
logic queue_full [NUM_PIPELINES-1:0];

// PE阵列相关
logic [DATA_WIDTH-1:0] pe_result [NUM_PIPELINES-1:0];
logic pe_busy [NUM_PIPELINES-1:0];
logic pe_idle [NUM_PIPELINES-1:0];

// 工作窃取控制器
work_stealing_controller #(
    .NUM_PIPELINES(NUM_PIPELINES)
) u_stealing_ctrl (
    .clk(clk),
    .rst_n(rst_n),
    .queue_empty(queue_empty),
    .queue_size(queue_size),
    .pe_idle(pe_idle),
    .steal_enable(steal_en),
    .src_queue(src_queue),
    .dst_queue(dst_queue),
    .steal_valid(steal_valid)
);

// 冗余PE映射器
redundant_pe_mapper #(
    .NUM_PIPELINES(NUM_PIPELINES),
    .NUM_REDUNDANT_PE(8)
) u_pe_mapper (
    .clk(clk),
    .rst_n(rst_n),
    .pe_busy(pe_busy),
    .queue_full(queue_full),
    .steal_failed(steal_failed),
    .redundant_pe_enable(redundant_pe_en),
    .pe_mapping(pe_mapping)
);

// 动态调整单元
dynamic_adjustment_unit #(
    .HISTORY_DEPTH(8)
) u_dau (
    .clk(clk),
    .rst_n(rst_n),
    .iter_history(iter_history),
    .converge_rate(converge_rate),
    .threshold_adj(threshold_adj)
);

// 重排序缓冲区
reorder_buffer #(
    .BUFFER_SIZE(32),
    .DATA_WIDTH(DATA_WIDTH)
) u_rob (
    .clk(clk),
    .rst_n(rst_n),
    .result_in(pe_result),
    .result_valid(pe_valid),
    .block_id(block_id),
    .vector_out(vector_out),
    .output_valid(output_valid)
);

// 状态机主控
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        current_state <= IDLE;
        iter_count <= 8'h0;
    end else begin
        current_state <= next_state;
        
        if (current_state == COMPUTE && compute_done)
            iter_count <= iter_count + 1;
        else if (current_state == IDLE)
            iter_count <= 8'h0;
    end
end

always_comb begin
    next_state = current_state;
    case (current_state)
        IDLE: if (matrix_valid) next_state = INIT;
        INIT: next_state = BLOCK_PARTITION;
        BLOCK_PARTITION: if (partition_done) next_state = SCHEDULE;
        SCHEDULE: if (schedule_done) next_state = COMPUTE;
        COMPUTE: if (compute_done) next_state = CONVERGE_CHECK;
        CONVERGE_CHECK: begin
            if (converge) next_state = DONE_STATE;
            else if (iter_count >= MAX_ITER) next_state = DONE_STATE;
            else next_state = SCHEDULE;
        end
        DONE_STATE: if (output_valid) next_state = IDLE;
        default: next_state = IDLE;
    endcase
end

// 输出信号
assign done = (current_state == DONE_STATE);
assign error = (iter_count >= MAX_ITER && !converge);

endmodule