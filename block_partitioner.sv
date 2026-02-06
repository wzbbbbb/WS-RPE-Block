`timescale 1ns/1ps

module block_partitioner #(
    parameter MATRIX_SIZE = 32,
    parameter DATA_WIDTH = 32,
    parameter MAX_BLOCKS = 16  // MATRIX_SIZE/2
)(
    input logic clk,
    input logic rst_n,
    
    // 矩阵输入
    input logic [DATA_WIDTH-1:0] matrix_in [MATRIX_SIZE-1:0][MATRIX_SIZE-1:0],
    input logic matrix_valid,
    input logic [7:0] matrix_dim,  // 实际矩阵维度
    
    // 分区配置
    input logic [2:0] partition_mode,  // 0: 2x2, 1: 4x4, 2: 自适应
    input logic enable_diagonal_only,
    input logic [3:0] block_stride,
    
    // 分区输出
    output logic [DATA_WIDTH-1:0] block_data [MAX_BLOCKS-1:0][1:0][1:0],
    output logic [7:0] block_size [MAX_BLOCKS-1:0],
    output logic block_valid [MAX_BLOCKS-1:0],
    output logic [MAX_BLOCKS-1:0] diagonal_mask,
    output logic partition_done,
    
    // 统计信息
    output logic [7:0] num_blocks,
    output logic [7:0] blocks_per_row,
    output logic [15:0] total_elements,
    output logic [7:0] sparsity_ratio
);

// 状态定义
typedef enum logic [2:0] {
    PART_IDLE = 3'b000,
    PART_CALC_SPARSITY = 3'b001,
    PART_DETERMINE_MODE = 3'b010,
    PART_PARTITION_2x2 = 3'b011,
    PART_PARTITION_4x4 = 3'b100,
    PART_PARTITION_ADAPT = 3'b101,
    PART_MARK_DIAGONAL = 3'b110,
    PART_DONE = 3'b111
} part_state_t;

part_state_t state;

// 计数器
logic [7:0] row_counter;
logic [7:0] col_counter;
logic [7:0] block_counter;
logic [7:0] element_counter;
logic [15:0] zero_counter;
logic [7:0] stride_counter;

// 稀疏度计算
logic [7:0] current_sparsity;
logic diagonal_dominant;

// 块大小计算
logic [3:0] block_size_lut [MAX_BLOCKS-1:0];
logic [MAX_BLOCKS-1:0] block_active;

// 自适应分区相关
logic [7:0] energy_map [MATRIX_SIZE/2-1:0][MATRIX_SIZE/2-1:0];
logic [15:0] block_energy [MAX_BLOCKS-1:0];
logic [3:0] adaptive_block_size [MAX_BLOCKS-1:0];

// 初始化
initial begin
    for (int i = 0; i < MAX_BLOCKS; i++) begin
        block_valid[i] = 0;
        block_size[i] = 0;
        diagonal_mask[i] = 0;
        block_active[i] = 0;
        block_energy[i] = 0;
        adaptive_block_size[i] = 0;
        
        for (int j = 0; j < 2; j++) begin
            for (int k = 0; k < 2; k++) begin
                block_data[i][j][k] = 0;
            end
        }
    end
end

// 主状态机
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state <= PART_IDLE;
        row_counter <= 0;
        col_counter <= 0;
        block_counter <= 0;
        element_counter <= 0;
        zero_counter <= 0;
        stride_counter <= 0;
        partition_done <= 0;
        num_blocks <= 0;
        blocks_per_row <= 0;
        total_elements <= 0;
        sparsity_ratio <= 0;
    end else begin
        case (state)
            PART_IDLE: begin
                if (matrix_valid) begin
                    state <= PART_CALC_SPARSITY;
                    row_counter <= 0;
                    col_counter <= 0;
                    zero_counter <= 0;
                    element_counter <= 0;
                end
            end
            
            PART_CALC_SPARSITY: begin
                if (row_counter < matrix_dim) begin
                    if (col_counter < matrix_dim) begin
                        // 计算零元素数量
                        if (matrix_in[row_counter][col_counter] == 0) begin
                            zero_counter <= zero_counter + 1;
                        end
                        
                        element_counter <= element_counter + 1;
                        col_counter <= col_counter + 1;
                    end else begin
                        col_counter <= 0;
                        row_counter <= row_counter + 1;
                    end
                end else begin
                    // 计算稀疏度百分比
                    if (element_counter > 0) begin
                        sparsity_ratio <= (zero_counter * 100) / (matrix_dim * matrix_dim);
                    end
                    
                    // 检查是否对角占优
                    check_diagonal_dominance();
                    
                    state <= PART_DETERMINE_MODE;
                end
            end
            
            PART_DETERMINE_MODE: begin
                // 根据稀疏度和配置决定分区模式
                determine_partition_mode();
                state <= (partition_mode == 2'b00) ? PART_PARTITION_2x2 :
                        (partition_mode == 2'b01) ? PART_PARTITION_4x4 :
                        PART_PARTITION_ADAPT;
                
                row_counter <= 0;
                col_counter <= 0;
                block_counter <= 0;
            end
            
            PART_PARTITION_2x2: begin
                // 执行2x2分块
                if (row_counter < matrix_dim) begin
                    if (col_counter < matrix_dim) begin
                        // 提取2x2子块
                        extract_2x2_block(row_counter, col_counter, block_counter);
                        
                        block_valid[block_counter] <= 1;
                        block_size[block_counter] <= 2;
                        block_active[block_counter] <= 1;
                        
                        // 检查是否是对角块
                        if (row_counter == col_counter) begin
                            diagonal_mask[block_counter] <= 1;
                        end
                        
                        block_counter <= block_counter + 1;
                        col_counter <= col_counter + 2;
                    end else begin
                        col_counter <= 0;
                        row_counter <= row_counter + 2;
                    end
                end else begin
                    num_blocks <= block_counter;
                    blocks_per_row <= matrix_dim / 2;
                    total_elements <= matrix_dim * matrix_dim;
                    state <= PART_MARK_DIAGONAL;
                end
            end
            
            PART_PARTITION_4x4: begin
                // 执行4x4分块
                if (row_counter < matrix_dim) begin
                    if (col_counter < matrix_dim) begin
                        // 检查边界条件
                        logic [7:0] actual_rows, actual_cols;
                        actual_rows = (row_counter + 4 <= matrix_dim) ? 4 : (matrix_dim - row_counter);
                        actual_cols = (col_counter + 4 <= matrix_dim) ? 4 : (matrix_dim - col_counter);
                        
                        // 提取4x4子块（可能部分填充）
                        extract_nxn_block(row_counter, col_counter, block_counter, 
                                        actual_rows, actual_cols);
                        
                        block_valid[block_counter] <= 1;
                        block_size[block_counter] <= (actual_rows > actual_cols) ? actual_rows : actual_cols;
                        block_active[block_counter] <= 1;
                        
                        // 检查是否包含对角线
                        if (row_counter <= col_counter && col_counter < row_counter + 4 &&
                            col_counter <= row_counter && row_counter < col_counter + 4) begin
                            diagonal_mask[block_counter] <= 1;
                        }
                        
                        block_counter <= block_counter + 1;
                        col_counter <= col_counter + 4;
                    end else begin
                        col_counter <= 0;
                        row_counter <= row_counter + 4;
                    end
                end else begin
                    num_blocks <= block_counter;
                    blocks_per_row <= (matrix_dim + 3) / 4; // 向上取整
                    total_elements <= matrix_dim * matrix_dim;
                    state <= PART_MARK_DIAGONAL;
                end
            end
            
            PART_PARTITION_ADAPT: begin
                // 自适应分区（基于能量）
                if (stride_counter == 0) begin
                    // 计算2x2块的能量
                    calculate_block_energy();
                    stride_counter <= 1;
                end else if (stride_counter == 1) begin
                    // 合并高能量块
                    merge_high_energy_blocks();
                    stride_counter <= 2;
                end else begin
                    // 生成最终分区
                    generate_adaptive_partition();
                    state <= PART_MARK_DIAGONAL;
                end
            end
            
            PART_MARK_DIAGONAL: begin
                if (enable_diagonal_only) begin
                    // 只保留对角线块
                    for (int i = 0; i < MAX_BLOCKS; i++) begin
                        if (!diagonal_mask[i]) begin
                            block_valid[i] <= 0;
                            block_active[i] <= 0;
                        end
                    end
                end
                
                state <= PART_DONE;
            end
            
            PART_DONE: begin
                partition_done <= 1;
                if (block_counter > 0) begin
                    // 输出最终块数
                    num_blocks <= block_counter;
                end
                
                // 返回空闲状态
                state <= PART_IDLE;
                partition_done <= 0;
            end
        endcase
    end
end

// 提取2x2块
task automatic extract_2x2_block;
    input [7:0] start_row;
    input [7:0] start_col;
    input integer block_id;
    begin
        // 提取四个元素
        block_data[block_id][0][0] = matrix_in[start_row][start_col];
        block_data[block_id][0][1] = (start_col + 1 < matrix_dim) ? 
                                    matrix_in[start_row][start_col + 1] : 0;
        block_data[block_id][1][0] = (start_row + 1 < matrix_dim) ? 
                                    matrix_in[start_row + 1][start_col] : 0;
        block_data[block_id][1][1] = (start_row + 1 < matrix_dim && start_col + 1 < matrix_dim) ? 
                                    matrix_in[start_row + 1][start_col + 1] : 0;
    end
endtask

// 提取NxN块
task automatic extract_nxn_block;
    input [7:0] start_row;
    input [7:0] start_col;
    input integer block_id;
    input [7:0] rows;
    input [7:0] cols;
    integer i, j;
    begin
        // 初始化块数据
        for (i = 0; i < 4; i++) begin
            for (j = 0; j < 4; j++) begin
                if (i < rows && j < cols) begin
                    // 使用2x2存储结构，扩展到4x4需要映射
                    if (i < 2 && j < 2) begin
                        block_data[block_id][i][j] = matrix_in[start_row + i][start_col + j];
                    end
                end else begin
                    // 超出部分填零
                    if (i < 2 && j < 2) begin
                        block_data[block_id][i][j] = 0;
                    end
                end
            end
        end
    end
endtask

// 检查对角占优
task automatic check_diagonal_dominance;
    integer i, j;
    logic [DATA_WIDTH-1:0] diag_sum, off_diag_sum;
    begin
        diag_sum = 0;
        off_diag_sum = 0;
        
        for (i = 0; i < matrix_dim; i++) begin
            for (j = 0; j < matrix_dim; j++) begin
                if (i == j) begin
                    diag_sum = diag_sum + absolute_value(matrix_in[i][j]);
                end else begin
                    off_diag_sum = off_diag_sum + absolute_value(matrix_in[i][j]);
                end
            end
        end
        
        // 如果对角线元素之和大于非对角线元素之和，则是对角占优
        diagonal_dominant <= (diag_sum > off_diag_sum);
    end
endtask

// 决定分区模式
task automatic determine_partition_mode;
    begin
        if (partition_mode == 2'b10) begin // 自适应模式
            // 基于稀疏度选择
            if (sparsity_ratio > 80) begin // 非常稀疏
                // 使用2x2分块以捕获稀疏结构
                partition_mode <= 2'b00;
            end else if (sparsity_ratio > 50) begin // 中等稀疏
                // 使用混合大小
                partition_mode <= 2'b10;
            end else begin // 密集
                // 使用4x4分块以提高吞吐量
                partition_mode <= 2'b01;
            end
        end
        
        // 如果是对角占优矩阵，优先使用2x2分块
        if (diagonal_dominant) begin
            partition_mode <= 2'b00;
        end
    end
endtask

// 计算块能量
task automatic calculate_block_energy;
    integer i, j, block_idx;
    begin
        block_idx = 0;
        
        for (i = 0; i < matrix_dim; i = i + 2) begin
            for (j = 0; j < matrix_dim; j = j + 2) begin
                // 计算2x2块的Frobenius范数（能量）
                logic [DATA_WIDTH-1:0] energy;
                energy = absolute_value(matrix_in[i][j]) * absolute_value(matrix_in[i][j]) +
                        absolute_value(matrix_in[i][j+1]) * absolute_value(matrix_in[i][j+1]) +
                        absolute_value(matrix_in[i+1][j]) * absolute_value(matrix_in[i+1][j]) +
                        absolute_value(matrix_in[i+1][j+1]) * absolute_value(matrix_in[i+1][j+1]);
                
                energy_map[i/2][j/2] <= energy;
                block_energy[block_idx] <= energy;
                
                block_idx = block_idx + 1;
            end
        end
    end
endtask

// 合并高能量块
task automatic merge_high_energy_blocks;
    integer i, j, block_idx;
    logic [7:0] merge_map [MAX_BLOCKS-1:0];
    begin
        block_idx = 0;
        
        // 初始化合并映射
        for (i = 0; i < MAX_BLOCKS; i++) begin
            merge_map[i] = i; // 初始每个块独立
        end
        
        // 遍历块，合并相邻的高能量块
        for (i = 0; i < matrix_dim/2; i++) begin
            for (j = 0; j < matrix_dim/2; j++) begin
                integer idx = i * (matrix_dim/2) + j;
                
                // 检查右侧邻居
                if (j + 1 < matrix_dim/2) begin
                    integer right_idx = i * (matrix_dim/2) + (j + 1);
                    if (block_energy[idx] > threshold && 
                        block_energy[right_idx] > threshold) begin
                        // 合并这两个块
                        merge_map[right_idx] = idx;
                        adaptive_block_size[idx] <= 4; // 2x4块
                    end
                end
                
                // 检查下侧邻居
                if (i + 1 < matrix_dim/2) begin
                    integer down_idx = (i + 1) * (matrix_dim/2) + j;
                    if (block_energy[idx] > threshold && 
                        block_energy[down_idx] > threshold) begin
                        // 合并这两个块
                        merge_map[down_idx] = idx;
                        adaptive_block_size[idx] <= 4; // 4x2块
                    end
                end
            end
        end
    end
endtask

// 生成自适应分区
task automatic generate_adaptive_partition;
    integer i, j, output_idx;
    logic [7:0] processed [MAX_BLOCKS-1:0];
    begin
        output_idx = 0;
        
        for (i = 0; i < MAX_BLOCKS; i++) begin
            processed[i] = 0;
        end
        
        for (i = 0; i < matrix_dim/2; i++) begin
            for (j = 0; j < matrix_dim/2; j++) begin
                integer idx = i * (matrix_dim/2) + j;
                
                if (!processed[idx]) begin
                    if (adaptive_block_size[idx] == 4) begin
                        // 合并块，提取4x4区域
                        extract_merged_block(i, j, output_idx, adaptive_block_size[idx]);
                        block_size[output_idx] <= 4;
                    end else begin
                        // 标准2x2块
                        extract_2x2_block(i*2, j*2, output_idx);
                        block_size[output_idx] <= 2;
                    end
                    
                    block_valid[output_idx] <= 1;
                    block_active[output_idx] <= 1;
                    
                    // 标记已处理
                    processed[idx] <= 1;
                    if (adaptive_block_size[idx] == 4) begin
                        // 标记合并的邻居块
                        integer neighbor_idx;
                        if (j + 1 < matrix_dim/2) begin
                            neighbor_idx = i * (matrix_dim/2) + (j + 1);
                            processed[neighbor_idx] <= 1;
                        end
                        if (i + 1 < matrix_dim/2) begin
                            neighbor_idx = (i + 1) * (matrix_dim/2) + j;
                            processed[neighbor_idx] <= 1;
                        end
                    end
                    
                    output_idx = output_idx + 1;
                end
            end
        end
        
        num_blocks <= output_idx;
        block_counter <= output_idx;
    end
endtask

// 提取合并块
task automatic extract_merged_block;
    input integer start_i;
    input integer start_j;
    input integer block_id;
    input [3:0] size;
    integer row, col, i, j;
    begin
        row = start_i * 2;
        col = start_j * 2;
        
        if (size == 4) begin // 4x4块
            for (i = 0; i < 4; i++) begin
                for (j = 0; j < 4; j++) begin
                    if (i < 2 && j < 2) begin
                        if (row + i < matrix_dim && col + j < matrix_dim) begin
                            block_data[block_id][i][j] = matrix_in[row + i][col + j];
                        end else begin
                            block_data[block_id][i][j] = 0;
                        end
                    end
                end
            end
        end
    end
endtask

// 辅助函数：计算绝对值
function automatic [DATA_WIDTH-1:0] absolute_value;
    input [DATA_WIDTH-1:0] value;
    begin
        absolute_value = value[DATA_WIDTH-1] ? -value : value;
    end
endfunction

// 计算能量阈值
function automatic [DATA_WIDTH-1:0] calculate_threshold;
    logic [DATA_WIDTH-1:0] avg_energy, sum_energy;
    integer i, count;
    begin
        sum_energy = 0;
        count = 0;
        
        for (i = 0; i < MAX_BLOCKS; i++) begin
            if (block_active[i]) begin
                sum_energy = sum_energy + block_energy[i];
                count = count + 1;
            end
        end
        
        if (count > 0) begin
            avg_energy = sum_energy / count;
            calculate_threshold = avg_energy * 2; // 两倍平均能量
        end else begin
            calculate_threshold = 0;
        end
    end
endfunction

// 获取块的能量阈值
logic [DATA_WIDTH-1:0] threshold;
assign threshold = calculate_threshold();

// 计算块坐标
function automatic [7:0] get_block_row;
    input integer block_id;
    begin
        get_block_row = (block_id / blocks_per_row) * 2;
    end
endfunction

function automatic [7:0] get_block_col;
    input integer block_id;
    begin
        get_block_col = (block_id % blocks_per_row) * 2;
    end
endfunction

endmodule