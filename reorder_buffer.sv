`timescale 1ns/1ps

module reorder_buffer #(
    parameter BUFFER_SIZE = 32,
    parameter DATA_WIDTH = 32,
    parameter MAX_BLOCKS = 16,
    parameter VECTOR_SIZE = 32
)(
    input logic clk,
    input logic rst_n,
    
    // 结果输入接口
    input logic [DATA_WIDTH-1:0] result_in [MAX_BLOCKS-1:0][1:0],
    input logic result_valid [MAX_BLOCKS-1:0],
    input logic [7:0] block_id [MAX_BLOCKS-1:0],
    input logic [1:0] result_type [MAX_BLOCKS-1:0], // 0: 正常, 1: 冗余, 2: 窃取
    
    // 完成顺序控制
    input logic [MAX_BLOCKS-1:0] completion_mask,
    input logic [7:0] expected_sequence,
    input logic sequence_valid,
    
    // 收敛控制
    input logic convergence_start,
    input logic [7:0] iteration_number,
    
    // 输出接口
    output logic [DATA_WIDTH-1:0] vector_out [VECTOR_SIZE-1:0],
    output logic output_valid,
    output logic output_last,
    output logic [VECTOR_SIZE-1:0] vector_valid_mask,
    
    // 无穷范数计算
    output logic [DATA_WIDTH-1:0] infinity_norm,
    output logic norm_valid,
    
    // 向量归一化
    output logic [DATA_WIDTH-1:0] normalized_vector [VECTOR_SIZE-1:0],
    output logic normalization_done,
    
    // 缓冲区状态
    output logic [4:0] buffer_occupancy,
    output logic buffer_full,
    output logic buffer_empty,
    output logic [7:0] ready_blocks,
    
    // 性能监控
    output logic [15:0] reorder_stalls,
    output logic [15:0] out_of_order_count,
    output logic [7:0] buffer_efficiency
);

// 缓冲区条目结构
typedef struct packed {
    logic [DATA_WIDTH-1:0] data_0;
    logic [DATA_WIDTH-1:0] data_1;
    logic [7:0] block_id;
    logic [7:0] sequence;
    logic valid;
    logic ready;
    logic processed;
    logic [1:0] result_type;
    logic [7:0] iteration;
} buffer_entry_t;

buffer_entry_t buffer [BUFFER_SIZE-1:0];

// 指针管理
logic [4:0] write_ptr;
logic [4:0] read_ptr;
logic [4:0] commit_ptr;
logic [4:0] entries_count;

// 序列管理
logic [7:0] next_sequence;
logic [7:0] next_expected;
logic [7:0] max_sequence;

// 完成检测
logic [MAX_BLOCKS-1:0] block_completed;
logic [MAX_BLOCKS-1:0] contiguous_completed;
logic [7:0] completion_count;
logic contiguous_detected;

// 归一化状态
logic [4:0] norm_counter;
logic [DATA_WIDTH-1:0] max_value;
logic [DATA_WIDTH-1:0] current_max;
logic normalization_active;

// 输出状态
logic [4:0] output_counter;
logic output_active;

// 错误检测
logic sequence_error;
logic duplicate_error;
logic [3:0] error_code;

// 性能计数器
logic [15:0] stall_counter;
logic [15:0] ooo_counter;
logic [15:0] cycle_counter;

// 状态机
typedef enum logic [2:0] {
    ROB_IDLE = 3'b000,
    ROB_WRITE = 3'b001,
    ROB_CHECK = 3'b010,
    ROB_COMMIT = 3'b011,
    ROB_NORMALIZE = 3'b100,
    ROB_OUTPUT = 3'b101,
    ROB_ERROR = 3'b110
} rob_state_t;

rob_state_t state;

// 初始化
initial begin
    for (int i = 0; i < BUFFER_SIZE; i++) begin
        buffer[i].valid = 0;
        buffer[i].ready = 0;
        buffer[i].processed = 0;
        buffer[i].sequence = 0;
    end
end

// 主状态机
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state <= ROB_IDLE;
        write_ptr <= 0;
        read_ptr <= 0;
        commit_ptr <= 0;
        entries_count <= 0;
        next_sequence <= 0;
        next_expected <= 0;
        max_sequence <= 0;
        output_valid <= 0;
        output_last <= 0;
        norm_valid <= 0;
        normalization_done <= 0;
        completion_count <= 0;
        contiguous_detected <= 0;
        norm_counter <= 0;
        output_counter <= 0;
        output_active <= 0;
        normalization_active <= 0;
        stall_counter <= 0;
        ooo_counter <= 0;
        cycle_counter <= 0;
        error_code <= 0;
        sequence_error <= 0;
        duplicate_error <= 0;
        
        for (int i = 0; i < VECTOR_SIZE; i++) begin
            vector_out[i] <= 0;
            normalized_vector[i] <= 0;
            vector_valid_mask[i] <= 0;
        end
        
        infinity_norm <= 0;
        current_max <= 0;
        max_value <= 0;
    end else begin
        cycle_counter <= cycle_counter + 1;
        
        case (state)
            ROB_IDLE: begin
                if (|result_valid) begin
                    state <= ROB_WRITE;
                end else if (convergence_start) begin
                    state <= ROB_CHECK;
                end
            end
            
            ROB_WRITE: begin
                // 写入结果到缓冲区
                for (int i = 0; i < MAX_BLOCKS; i++) begin
                    if (result_valid[i] && entries_count < BUFFER_SIZE) begin
                        // 查找空闲缓冲区位置
                        integer free_slot = find_free_slot();
                        if (free_slot >= 0) begin
                            buffer[free_slot].data_0 <= result_in[i][0];
                            buffer[free_slot].data_1 <= result_in[i][1];
                            buffer[free_slot].block_id <= block_id[i];
                            buffer[free_slot].sequence <= next_sequence;
                            buffer[free_slot].valid <= 1;
                            buffer[free_slot].ready <= 0;
                            buffer[free_slot].processed <= 0;
                            buffer[free_slot].result_type <= result_type[i];
                            buffer[free_slot].iteration <= iteration_number;
                            
                            next_sequence <= next_sequence + 1;
                            entries_count <= entries_count + 1;
                            write_ptr <= (write_ptr + 1) % BUFFER_SIZE;
                            
                            // 更新最大序列号
                            if (next_sequence > max_sequence) begin
                                max_sequence <= next_sequence;
                            end
                        end else begin
                            // 缓冲区满，计数stall
                            stall_counter <= stall_counter + 1;
                        end
                    end
                end
                
                state <= ROB_CHECK;
            end
            
            ROB_CHECK: begin
                // 检查完成序列
                check_completion_sequence();
                
                if (contiguous_detected) begin
                    state <= ROB_COMMIT;
                end else if (entries_count >= BUFFER_SIZE/2) begin
                    // 缓冲区半满，强制提交最旧的条目
                    state <= ROB_COMMIT;
                end else begin
                    state <= ROB_IDLE;
                end
            end
            
            ROB_COMMIT: begin
                // 提交连续完成的结果
                if (read_ptr != commit_ptr) begin
                    integer idx = read_ptr;
                    
                    if (buffer[idx].valid && !buffer[idx].processed && 
                        buffer[idx].sequence <= next_expected) {
                        
                        // 提交结果到向量
                        commit_to_vector(idx);
                        
                        buffer[idx].processed <= 1;
                        read_ptr <= (read_ptr + 1) % BUFFER_SIZE;
                        
                        // 更新期望序列
                        next_expected <= next_expected + 1;
                        
                        // 如果这是最后一个连续块，开始归一化
                        if (buffer[idx].sequence == next_expected && 
                            !has_more_contiguous()) {
                            state <= ROB_NORMALIZE;
                            norm_counter <= 0;
                            current_max <= 0;
                            normalization_active <= 1;
                        }
                    } else if (buffer[idx].valid && buffer[idx].sequence > next_expected) {
                        // 乱序到达，计数
                        ooo_counter <= ooo_counter + 1;
                        out_of_order_count <= ooo_counter;
                        
                        // 尝试查找可提交的旧序列
                        integer older_idx = find_older_sequence(next_expected);
                        if (older_idx >= 0) begin
                            read_ptr <= older_idx;
                        end else begin
                            // 无法提交，返回检查
                            state <= ROB_CHECK;
                        end
                    }
                end else begin
                    state <= ROB_IDLE;
                end
            end
            
            ROB_NORMALIZE: begin
                if (norm_counter < VECTOR_SIZE) begin
                    if (vector_valid_mask[norm_counter]) begin
                        // 查找最大值
                        logic [DATA_WIDTH-1:0] abs_val;
                        abs_val = absolute_value(vector_out[norm_counter]);
                        
                        if (abs_val > current_max) begin
                            current_max <= abs_val;
                        end
                        
                        norm_counter <= norm_counter + 1;
                    end
                end else begin
                    // 完成最大值查找
                    max_value <= current_max;
                    
                    if (current_max > 0) begin
                        // 执行归一化
                        perform_normalization(current_max);
                    end else begin
                        // 全零向量，直接复制
                        normalized_vector <= vector_out;
                    end
                    
                    norm_valid <= 1;
                    infinity_norm <= current_max;
                    normalization_done <= 1;
                    
                    state <= ROB_OUTPUT;
                    output_counter <= 0;
                    output_active <= 1;
                end
            end
            
            ROB_OUTPUT: begin
                if (output_counter < VECTOR_SIZE) begin
                    // 输出归一化向量
                    if (output_counter == VECTOR_SIZE - 1) begin
                        output_last <= 1;
                    end
                    
                    output_valid <= 1;
                    output_counter <= output_counter + 1;
                end else begin
                    output_valid <= 0;
                    output_last <= 0;
                    output_active <= 0;
                    normalization_done <= 0;
                    norm_valid <= 0;
                    
                    // 清理已处理的缓冲区条目
                    cleanup_processed_entries();
                    
                    state <= ROB_IDLE;
                end
            end
            
            ROB_ERROR: begin
                // 错误处理状态
                handle_error_condition();
                
                if (cycle_counter[7:0] == 8'hFF) begin
                    state <= ROB_IDLE;
                end
            end
        endcase
        
        // 更新状态输出
        buffer_occupancy <= entries_count;
        buffer_full <= (entries_count == BUFFER_SIZE);
        buffer_empty <= (entries_count == 0);
        reorder_stalls <= stall_counter;
        
        // 计算缓冲区效率
        if (cycle_counter > 0) begin
            buffer_efficiency <= (cycle_counter - stall_counter) * 100 / cycle_counter;
        end
    end
end

// 查找空闲缓冲区槽位
function automatic integer find_free_slot;
    integer i, free_idx;
    logic found;
    begin
        free_idx = -1;
        found = 0;
        
        // 从写指针开始查找
        for (i = 0; i < BUFFER_SIZE; i++) begin
            integer idx = (write_ptr + i) % BUFFER_SIZE;
            
            if (!buffer[idx].valid) begin
                free_idx = idx;
                found = 1;
                break;
            end
        end
        
        // 如果没找到，查找整个缓冲区
        if (!found) begin
            for (i = 0; i < BUFFER_SIZE; i++) begin
                if (!buffer[i].valid) begin
                    free_idx = i;
                    break;
                end
            end
        end
        
        find_free_slot = free_idx;
    end
endfunction

// 检查完成序列
task automatic check_completion_sequence;
    integer i;
    logic [7:0] first_missing;
    logic found_gap;
    begin
        block_completed <= completion_mask;
        contiguous_completed <= 0;
        completion_count <= 0;
        contiguous_detected <= 0;
        first_missing = 255;
        found_gap = 0;
        
        // 从期望序列开始检查
        for (i = expected_sequence; i < MAX_BLOCKS; i++) begin
            if (block_completed[i]) begin
                if (!found_gap) begin
                    contiguous_completed[i] <= 1;
                    completion_count <= completion_count + 1;
                end
            end else begin
                if (!found_gap) begin
                    first_missing = i;
                    found_gap = 1;
                end
            end
        end
        
        // 如果有连续完成的块
        if (completion_count > 0) begin
            contiguous_detected <= 1;
        end
        
        // 更新就绪块数
        ready_blocks <= completion_count;
    end
endtask

// 提交到向量
task automatic commit_to_vector;
    input integer buffer_idx;
    integer vec_idx_0, vec_idx_1;
    begin
        // 计算向量索引
        vec_idx_0 = buffer[buffer_idx].block_id * 2;
        vec_idx_1 = vec_idx_0 + 1;
        
        if (vec_idx_0 < VECTOR_SIZE) begin
            vector_out[vec_idx_0] <= buffer[buffer_idx].data_0;
            vector_valid_mask[vec_idx_0] <= 1;
        end
        
        if (vec_idx_1 < VECTOR_SIZE) begin
            vector_out[vec_idx_1] <= buffer[buffer_idx].data_1;
            vector_valid_mask[vec_idx_1] <= 1;
        end
        
        // 标记块完成
        block_completed[buffer[buffer_idx].block_id] <= 1;
    end
endtask

// 检查是否有更多连续块
function automatic logic has_more_contiguous;
    integer i;
    logic has_more;
    begin
        has_more = 0;
        
        for (i = read_ptr; i != write_ptr; i = (i + 1) % BUFFER_SIZE) begin
            if (buffer[i].valid && !buffer[i].processed && 
                buffer[i].sequence == next_expected) {
                has_more = 1;
                break;
            end
        end
        
        has_more_contiguous = has_more;
    end
endfunction

// 查找更旧的序列
function automatic integer find_older_sequence;
    input [7:0] target_sequence;
    integer i, oldest_idx;
    logic [7:0] oldest_seq;
    begin
        oldest_idx = -1;
        oldest_seq = 255;
        
        for (i = 0; i < BUFFER_SIZE; i++) begin
            if (buffer[i].valid && !buffer[i].processed && 
                buffer[i].sequence < target_sequence && 
                buffer[i].sequence < oldest_seq) {
                oldest_idx = i;
                oldest_seq = buffer[i].sequence;
            end
        end
        
        find_older_sequence = oldest_idx;
    end
endfunction

// 执行归一化
task automatic perform_normalization;
    input [DATA_WIDTH-1:0] norm_factor;
    integer i;
    begin
        for (i = 0; i < VECTOR_SIZE; i++) begin
            if (vector_valid_mask[i]) begin
                normalized_vector[i] <= vector_out[i] / norm_factor;
            end else begin
                normalized_vector[i] <= 0;
            end
        end
    end
endtask

// 清理已处理的条目
task automatic cleanup_processed_entries;
    integer i;
    logic [4:0] new_count;
    begin
        new_count = 0;
        
        for (i = 0; i < BUFFER_SIZE; i++) begin
            if (buffer[i].processed) begin
                buffer[i].valid <= 0;
                buffer[i].ready <= 0;
                buffer[i].processed <= 0;
            end else if (buffer[i].valid) begin
                new_count = new_count + 1;
            end
        end
        
        entries_count <= new_count;
        
        // 重置向量有效掩码
        for (i = 0; i < VECTOR_SIZE; i++) begin
            vector_valid_mask[i] <= 0;
        end
    end
endtask

// 处理错误条件
task automatic handle_error_condition;
    begin
        // 检测序列错误
        if (next_sequence < max_sequence && 
            (next_sequence - max_sequence) > BUFFER_SIZE) {
            sequence_error <= 1;
            error_code <= 4'b0001;
        end
        
        // 检测重复块ID
        check_duplicate_blocks();
        
        if (duplicate_error) begin
            error_code <= 4'b0010;
        end
        
        // 根据错误代码采取行动
        case (error_code)
            4'b0001: begin // 序列错误
                // 重置序列计数
                next_sequence <= 0;
                max_sequence <= 0;
                next_expected <= 0;
            end
            
            4'b0010: begin // 重复块
                // 保留最新的，清除旧的
                remove_duplicate_blocks();
            end
            
            default: begin
                // 未知错误，清空缓冲区
                for (int i = 0; i < BUFFER_SIZE; i++) begin
                    buffer[i].valid <= 0;
                end
                entries_count <= 0;
            end
        endcase
    end
endtask

// 检查重复块
task automatic check_duplicate_blocks;
    integer i, j;
    begin
        duplicate_error <= 0;
        
        for (i = 0; i < BUFFER_SIZE; i++) begin
            if (buffer[i].valid) begin
                for (j = i + 1; j < BUFFER_SIZE; j++) begin
                    if (buffer[j].valid && 
                        buffer[i].block_id == buffer[j].block_id &&
                        buffer[i].iteration == buffer[j].iteration) {
                        duplicate_error <= 1;
                        break;
                    end
                end
            end
        end
    end
endtask

// 移除重复块
task automatic remove_duplicate_blocks;
    integer i, j;
    begin
        for (i = 0; i < BUFFER_SIZE; i++) begin
            if (buffer[i].valid) begin
                for (j = i + 1; j < BUFFER_SIZE; j++) begin
                    if (buffer[j].valid && 
                        buffer[i].block_id == buffer[j].block_id &&
                        buffer[i].iteration == buffer[j].iteration) {
                        // 保留序列号较大的
                        if (buffer[j].sequence > buffer[i].sequence) begin
                            buffer[i].valid <= 0;
                        end else begin
                            buffer[j].valid <= 0;
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

// 计算缓冲区利用率
function automatic [7:0] calculate_buffer_utilization;
    begin
        if (BUFFER_SIZE > 0) begin
            calculate_buffer_utilization = (entries_count * 100) / BUFFER_SIZE;
        end else begin
            calculate_buffer_utilization = 0;
        end
    end
endfunction

// 获取最旧的未处理条目
function automatic integer get_oldest_pending;
    integer i, oldest_idx;
    logic [7:0] oldest_seq;
    begin
        oldest_idx = -1;
        oldest_seq = 255;
        
        for (i = 0; i < BUFFER_SIZE; i++) begin
            if (buffer[i].valid && !buffer[i].processed && 
                buffer[i].sequence < oldest_seq) {
                oldest_idx = i;
                oldest_seq = buffer[i].sequence;
            end
        end
        
        get_oldest_pending = oldest_idx;
    end
endfunction

// 检查缓冲区健康状况
function automatic [3:0] check_buffer_health;
    logic [3:0] health_score;
    begin
        health_score = 4'b1111; // 初始满分
        
        // 检查利用率
        if (entries_count > BUFFER_SIZE * 3/4) begin
            health_score[0] = 0; // 缓冲区过满
        end
        
        // 检查stall次数
        if (stall_counter > 1000) begin
            health_score[1] = 0; // stall过多
        end
        
        // 检查乱序次数
        if (ooo_counter > 100) begin
            health_score[2] = 0; // 乱序过多
        end
        
        // 检查错误
        if (error_code != 0) begin
            health_score[3] = 0; // 有错误
        end
        
        check_buffer_health = health_score;
    end
endfunction

endmodule