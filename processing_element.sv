module processing_element #(
    parameter DATA_WIDTH = 32,
    parameter VECTOR_SIZE = 2
)(
    input logic clk,
    input logic rst_n,
    input logic enable,
    input logic [DATA_WIDTH-1:0] matrix_in [VECTOR_SIZE-1:0][VECTOR_SIZE-1:0],
    input logic [DATA_WIDTH-1:0] vector_in [VECTOR_SIZE-1:0],
    input logic [1:0] mode, // 0: normal, 1: redundant, 2: dual-path
    output logic [DATA_WIDTH-1:0] vector_out [VECTOR_SIZE-1:0],
    output logic done,
    output logic busy
);

// 内部寄存器
logic [DATA_WIDTH*VECTOR_SIZE-1:0] accumulator;
logic [2:0] state;
logic compute_done;

// 乘法累加阵列
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        for (int i = 0; i < VECTOR_SIZE; i++) begin
            vector_out[i] <= 0;
        end
        accumulator <= 0;
        state <= 0;
        busy <= 0;
    end else if (enable) begin
        busy <= 1;
        case (state)
            0: begin // 加载数据
                for (int i = 0; i < VECTOR_SIZE; i++) begin
                    accumulator[i*DATA_WIDTH +: DATA_WIDTH] <= vector_in[i];
                end
                state <= 1;
            end
            1: begin // 矩阵向量乘法
                for (int i = 0; i < VECTOR_SIZE; i++) begin
                    logic [DATA_WIDTH-1:0] sum;
                    sum = 0;
                    for (int j = 0; j < VECTOR_SIZE; j++) begin
                        sum = sum + matrix_in[i][j] * vector_in[j];
                    end
                    vector_out[i] <= sum;
                end
                state <= 2;
            end
            2: begin // 完成
                compute_done <= 1;
                busy <= 0;
                state <= 0;
            end
        endcase
    end else begin
        busy <= 0;
        compute_done <= 0;
    end
end

assign done = compute_done;

endmodule