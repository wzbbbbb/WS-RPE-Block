module work_stealing_controller #(
    parameter NUM_PIPELINES = 16
)(
    input logic clk,
    input logic rst_n,
    input logic queue_empty [NUM_PIPELINES-1:0],
    input logic [7:0] queue_size [NUM_PIPELINES-1:0],
    input logic pe_idle [NUM_PIPELINES-1:0],
    output logic steal_enable,
    output logic [$clog2(NUM_PIPELINES)-1:0] src_queue,
    output logic [$clog2(NUM_PIPELINES)-1:0] dst_queue,
    output logic steal_valid
);

// 内部寄存器
logic [7:0] max_queue_size;
logic [$clog2(NUM_PIPELINES)-1:0] max_queue_idx;
logic idle_detected;

// 查找最大队列
always_comb begin
    max_queue_size = 0;
    max_queue_idx = 0;
    for (int i = 0; i < NUM_PIPELINES; i++) begin
        if (queue_size[i] > max_queue_size && !queue_empty[i]) begin
            max_queue_size = queue_size[i];
            max_queue_idx = i;
        end
    end
end

// 检测空闲PE
always_comb begin
    idle_detected = 0;
    dst_queue = 0;
    for (int i = 0; i < NUM_PIPELINES; i++) begin
        if (pe_idle[i] && queue_empty[i]) begin
            idle_detected = 1;
            dst_queue = i;
            break;
        end
    end
end

// 窃取逻辑
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        steal_enable <= 0;
        steal_valid <= 0;
        src_queue <= 0;
    end else begin
        steal_enable <= idle_detected && (max_queue_size > 1);
        src_queue <= max_queue_idx;
        
        if (steal_enable && !queue_empty[max_queue_idx] && !queue_empty[dst_queue]) begin
            steal_valid <= 1;
        end else begin
            steal_valid <= 0;
        end
    end
end

endmodule