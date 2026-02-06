module axis_interface #(
    parameter DATA_WIDTH = 32,
    parameter MAX_SIZE = 32
)(
    // AXI-Stream接口
    input logic aclk,
    input logic aresetn,
    
    // 从设备接口
    input logic s_axis_tvalid,
    input logic [DATA_WIDTH-1:0] s_axis_tdata,
    input logic s_axis_tlast,
    output logic s_axis_tready,
    
    // 主设备接口
    output logic m_axis_tvalid,
    output logic [DATA_WIDTH-1:0] m_axis_tdata,
    output logic m_axis_tlast,
    input logic m_axis_tready,
    
    // 内部接口
    output logic [DATA_WIDTH-1:0] matrix_out [MAX_SIZE-1:0][MAX_SIZE-1:0],
    output logic [DATA_WIDTH-1:0] vector_out [MAX_SIZE-1:0],
    output logic data_valid,
    
    input logic [DATA_WIDTH-1:0] result_in [MAX_SIZE-1:0],
    input logic result_valid
);

// 输入FIFO
logic [DATA_WIDTH-1:0] input_fifo [$];
logic fifo_empty;
logic fifo_full;

// 状态机
typedef enum logic [1:0] {
    IDLE,
    RECEIVE_MATRIX,
    RECEIVE_VECTOR,
    SEND_RESULT
} state_t;

state_t current_state;

// 计数器
logic [7:0] row_count;
logic [7:0] col_count;
logic [7:0] element_count;

// 输入处理
always_ff @(posedge aclk or negedge aresetn) begin
    if (!aresetn) begin
        current_state <= IDLE;
        row_count <= 0;
        col_count <= 0;
        element_count <= 0;
        data_valid <= 0;
        s_axis_tready <= 1;
    end else begin
        case (current_state)
            IDLE: begin
                if (s_axis_tvalid) begin
                    current_state <= RECEIVE_MATRIX;
                    row_count <= 0;
                    col_count <= 0;
                end
            end
            
            RECEIVE_MATRIX: begin
                if (s_axis_tvalid && s_axis_tready) begin
                    matrix_out[row_count][col_count] <= s_axis_tdata;
                    col_count <= col_count + 1;
                    
                    if (col_count == MAX_SIZE-1) begin
                        col_count <= 0;
                        row_count <= row_count + 1;
                    end
                    
                    if (s_axis_tlast) begin
                        current_state <= RECEIVE_VECTOR;
                        element_count <= 0;
                    end
                end
            end
            
            RECEIVE_VECTOR: begin
                if (s_axis_tvalid && s_axis_tready) begin
                    vector_out[element_count] <= s_axis_tdata;
                    element_count <= element_count + 1;
                    
                    if (s_axis_tlast) begin
                        data_valid <= 1;
                        current_state <= IDLE;
                    end
                end
            end
            
            default: current_state <= IDLE;
        endcase
        
        if (data_valid) data_valid <= 0;
    end
end

// 输出处理
always_ff @(posedge aclk or negedge aresetn) begin
    if (!aresetn) begin
        m_axis_tvalid <= 0;
        m_axis_tlast <= 0;
        m_axis_tdata <= 0;
    end else if (result_valid) begin
        // 发送特征向量
        for (int i = 0; i < MAX_SIZE; i++) begin
            if (m_axis_tready) begin
                m_axis_tvalid <= 1;
                m_axis_tdata <= result_in[i];
                m_axis_tlast <= (i == MAX_SIZE-1);
            end
        end
    end else begin
        m_axis_tvalid <= 0;
        m_axis_tlast <= 0;
    end
end

endmodule