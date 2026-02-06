`timescale 1ns/1ps

module dynamic_adjustment_unit #(
    parameter HISTORY_DEPTH = 8,
    parameter DATA_WIDTH = 32,
    parameter EWMA_ALPHA = 32'h3E4CCCCD,  // 0.2
    parameter EWMA_BETA = 32'h3F4CCCCD    // 0.8
)(
    input logic clk,
    input logic rst_n,
    
    // 迭代历史输入
    input logic [7:0] iter_history [HISTORY_DEPTH-1:0],
    input logic history_valid,
    input logic [7:0] current_iteration,
    input logic iteration_complete,
    
    // 矩阵特征
    input logic [7:0] matrix_sparsity,
    input logic diagonal_dominant,
    input logic [3:0] matrix_condition,  // 条件数估计
    input logic [2:0] matrix_structure,
    
    // 误差输入
    input logic [DATA_WIDTH-1:0] error_vector [15:0],
    input logic [DATA_WIDTH-1:0] max_error,
    input logic error_valid,
    
    // 性能反馈
    input logic [15:0] performance_metrics [7:0],
    input logic performance_valid,
    
    // 系统状态
    input logic system_congested,
    input logic [3:0] workload_level,
    input logic deadline_approaching,
    
    // 调整输出
    output logic [DATA_WIDTH-1:0] threshold_adjusted,
    output logic threshold_valid,
    output logic [3:0] convergence_mode,
    output logic [2:0] iteration_limit_mode,
    
    // 预测输出
    output logic [7:0] predicted_iterations,
    output logic prediction_valid,
    output logic [7:0] confidence_level,
    
    // 自适应参数
    output logic [DATA_WIDTH-1:0] learning_rate,
    output logic [3:0] aggressiveness,
    output logic enable_early_stop,
    
    // 统计输出
    output logic [15:0] total_adjustments,
    output logic [15:0] successful_predictions,
    output logic [15:0] prediction_accuracy,
    output logic [7:0] adaptation_rate
);

// 历史缓冲区
logic [7:0] history_buffer [HISTORY_DEPTH-1:0];
logic [3:0] history_ptr;
logic [7:0] history_count;

// 误差历史
logic [DATA_WIDTH-1:0] error_history [HISTORY_DEPTH-1:0];
logic [3:0] error_ptr;

// 指数加权移动平均
logic [DATA_WIDTH-1:0] ewma_iterations;
logic [DATA_WIDTH-1:0] ewma_error;
logic [DATA_WIDTH-1:0] ewma_performance;

// 预测模型
logic [DATA_WIDTH-1:0] base_threshold;
logic [DATA_WIDTH-1:0] adaptive_factor;
logic [DATA_WIDTH-1:0] sparsity_factor;
logic [DATA_WIDTH-1:0] workload_factor;

// 学习模型参数
logic [DATA_WIDTH-1:0] weight_matrix [3:0][3:0];
logic [DATA_WIDTH-1:0] bias_vector [3:0];
logic [DATA_WIDTH-1:0] model_output [3:0];

// 状态机
typedef enum logic [2:0] {
    DAU_IDLE = 3'b000,
    DAU_COLLECT = 3'b001,
    DAU_TRAIN = 3'b010,
    DAU_PREDICT = 3'b011,
    DAU_ADJUST = 3'b100,
    DAU_EVALUATE = 3'b101,
    DAU_UPDATE = 3'b110
} dau_state_t;

dau_state_t state;

// 计数器和寄存器
logic [7:0] collect_counter;
logic [7:0] train_counter;
logic [7:0] adjust_counter;
logic [15:0] cycle_counter;
logic [7:0] adaptation_counter;

// 性能跟踪
logic [15:0] success_counter;
logic [15:0] total_predictions;
logic [7:0] current_accuracy;

// 收敛状态
logic [3:0] convergence_state;
logic [7:0] convergence_speed;
logic [1:0] convergence_trend;  // 0: 加速, 1: 减速, 2: 振荡

// 初始化
initial begin
    // 初始化历史缓冲区
    for (int i = 0; i < HISTORY_DEPTH; i++) begin
        history_buffer[i] = 0;
        error_history[i] = 0;
    end
    
    // 初始化权重矩阵（单位矩阵）
    for (int i = 0; i < 4; i++) begin
        for (int j = 0; j < 4; j++) begin
            weight_matrix[i][j] = (i == j) ? 32'h3F800000 : 0; // 1.0 on diagonal
        end
        bias_vector[i] = 0;
    end
    
    // 初始化EWMA
    ewma_iterations = 32'h42000000; // 初始估计: 32次迭代
    ewma_error = 32'h3DCCCCCD;      // 初始误差: 0.1
    ewma_performance = 32'h3F000000; // 初始性能: 0.5
    
    base_threshold = 32'h3A83126F;   // 1e-3
end

// 主状态机
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state <= DAU_IDLE;
        history_ptr <= 0;
        error_ptr <= 0;
        history_count <= 0;
        collect_counter <= 0;
        train_counter <= 0;
        adjust_counter <= 0;
        cycle_counter <= 0;
        adaptation_counter <= 0;
        success_counter <= 0;
        total_predictions <= 0;
        current_accuracy <= 0;
        convergence_state <= 0;
        convergence_speed <= 0;
        convergence_trend <= 0;
        
        threshold_adjusted <= base_threshold;
        threshold_valid <= 0;
        convergence_mode <= 4'b0010; // 中等严格度
        iteration_limit_mode <= 3'b010; // 中等限制
        predicted_iterations <= 32;
        prediction_valid <= 0;
        confidence_level <= 50;
        learning_rate <= EWMA_ALPHA;
        aggressiveness <= 4'b0100;
        enable_early_stop <= 1;
        
        total_adjustments <= 0;
        successful_predictions <= 0;
        prediction_accuracy <= 0;
        adaptation_rate <= 0;
    end else begin
        cycle_counter <= cycle_counter + 1;
        
        case (state)
            DAU_IDLE: begin
                if (history_valid || iteration_complete) begin
                    state <= DAU_COLLECT;
                    collect_counter <= 0;
                end
            end
            
            DAU_COLLECT: begin
                // 收集历史和误差数据
                if (collect_counter < HISTORY_DEPTH) begin
                    if (history_valid) begin
                        history_buffer[history_ptr] <= iter_history[collect_counter];
                        history_ptr <= (history_ptr + 1) % HISTORY_DEPTH;
                        history_count <= (history_count < HISTORY_DEPTH) ? 
                                       history_count + 1 : HISTORY_DEPTH;
                    end
                    
                    if (error_valid) begin
                        error_history[error_ptr] <= max_error;
                        error_ptr <= (error_ptr + 1) % HISTORY_DEPTH;
                    end
                    
                    collect_counter <= collect_counter + 1;
                end else begin
                    state <= DAU_TRAIN;
                    train_counter <= 0;
                end
            end
            
            DAU_TRAIN: begin
                // 训练预测模型
                if (train_counter < 4) begin
                    // 更新指数加权移动平均
                    update_ewma_models();
                    
                    // 更新学习模型参数
                    update_learning_model();
                    
                    train_counter <= train_counter + 1;
                end else begin
                    state <= DAU_PREDICT;
                end
            end
            
            DAU_PREDICT: begin
                // 生成预测
                generate_predictions();
                
                // 计算置信度
                calculate_confidence();
                
                prediction_valid <= 1;
                state <= DAU_ADJUST;
                adjust_counter <= 0;
            end
            
            DAU_ADJUST: begin
                // 根据预测调整参数
                if (adjust_counter == 0) begin
                    adjust_convergence_threshold();
                    adjust_iteration_limits();
                    adjust_learning_parameters();
                    
                    threshold_valid <= 1;
                    adjust_counter <= 1;
                end else begin
                    state <= DAU_EVALUATE;
                    threshold_valid <= 0;
                end
            end
            
            DAU_EVALUATE: begin
                // 评估调整效果
                evaluate_adjustment_effectiveness();
                
                // 更新成功率
                update_success_metrics();
                
                state <= DAU_UPDATE;
            end
            
            DAU_UPDATE: begin
                // 更新统计信息
                update_statistics();
                
                // 自适应调整学习率
                adapt_learning_rate();
                
                state <= DAU_IDLE;
                adaptation_counter <= adaptation_counter + 1;
                total_adjustments <= total_adjustments + 1;
            end
        endcase
        
        // 更新适应率
        if (cycle_counter > 0) begin
            adaptation_rate <= (adaptation_counter * 100) / (cycle_counter >> 8);
        end
    end
end

// 更新EWMA模型
task automatic update_ewma_models;
    logic [DATA_WIDTH-1:0] avg_iterations;
    logic [DATA_WIDTH-1:0] avg_error;
    logic [DATA_WIDTH-1:0] avg_performance;
    begin
        // 计算平均迭代次数
        if (history_count > 0) begin
            avg_iterations = calculate_average_iterations();
            ewma_iterations <= EWMA_ALPHA * avg_iterations + 
                             EWMA_BETA * ewma_iterations;
        end
        
        // 计算平均误差
        if (error_ptr > 0) begin
            avg_error = calculate_average_error();
            ewma_error <= EWMA_ALPHA * avg_error + 
                         EWMA_BETA * ewma_error;
        end
        
        // 计算平均性能
        if (performance_valid) begin
            avg_performance = calculate_average_performance();
            ewma_performance <= EWMA_ALPHA * avg_performance + 
                              EWMA_BETA * ewma_performance;
        end
    end
endtask

// 计算平均迭代次数
function automatic [DATA_WIDTH-1:0] calculate_average_iterations;
    logic [DATA_WIDTH-1:0] sum;
    integer i, count;
    begin
        sum = 0;
        count = 0;
        
        for (i = 0; i < HISTORY_DEPTH; i++) begin
            if (i < history_count) begin
                sum = sum + history_buffer[i];
                count = count + 1;
            end
        end
        
        if (count > 0) begin
            calculate_average_iterations = sum / count;
        end else begin
            calculate_average_iterations = 32'h42000000; // 默认32
        end
    end
endfunction

// 计算平均误差
function automatic [DATA_WIDTH-1:0] calculate_average_error;
    logic [DATA_WIDTH-1:0] sum;
    integer i, count;
    begin
        sum = 0;
        count = 0;
        
        for (i = 0; i < HISTORY_DEPTH; i++) begin
            if (error_history[i] != 0) begin
                sum = sum + error_history[i];
                count = count + 1;
            end
        end
        
        if (count > 0) begin
            calculate_average_error = sum / count;
        end else begin
            calculate_average_error = 32'h3DCCCCCD; // 默认0.1
        end
    end
endfunction

// 计算平均性能
function automatic [DATA_WIDTH-1:0] calculate_average_performance;
    logic [DATA_WIDTH-1:0] sum;
    integer i;
    begin
        sum = 0;
        
        for (i = 0; i < 8; i++) begin
            sum = sum + performance_metrics[i];
        end
        
        calculate_average_performance = sum / 8;
    end
endfunction

// 更新学习模型
task automatic update_learning_model;
    logic [DATA_WIDTH-1:0] gradient [3:0][3:0];
    logic [DATA_WIDTH-1:0] error_gradient [3:0];
    integer i, j;
    begin
        // 计算梯度
        calculate_gradients(gradient, error_gradient);
        
        // 更新权重矩阵
        for (i = 0; i < 4; i++) begin
            for (j = 0; j < 4; j++) begin
                weight_matrix[i][j] <= weight_matrix[i][j] - 
                                      learning_rate * gradient[i][j];
            end
            bias_vector[i] <= bias_vector[i] - 
                            learning_rate * error_gradient[i];
        end
    end
endtask

// 计算梯度
task automatic calculate_gradients;
    output [DATA_WIDTH-1:0] grad_matrix [3:0][3:0];
    output [DATA_WIDTH-1:0] grad_bias [3:0];
    logic [DATA_WIDTH-1:0] prediction_error [3:0];
    integer i, j;
    begin
        // 计算预测误差
        for (i = 0; i < 4; i++) begin
            prediction_error[i] = model_output[i] - get_target_value(i);
        end
        
        // 计算权重梯度
        for (i = 0; i < 4; i++) begin
            for (j = 0; j < 4; j++) begin
                grad_matrix[i][j] = prediction_error[i] * get_feature_value(j);
            end
            grad_bias[i] = prediction_error[i];
        end
    end
endtask

// 获取特征值
function automatic [DATA_WIDTH-1:0] get_feature_value;
    input integer feature_idx;
    begin
        case (feature_idx)
            0: get_feature_value = ewma_iterations;
            1: get_feature_value = ewma_error;
            2: get_feature_value = matrix_sparsity / 100.0;
            3: get_feature_value = matrix_condition / 10.0;
            default: get_feature_value = 0;
        endcase
    end
endfunction

// 获取目标值
function automatic [DATA_WIDTH-1:0] get_target_value;
    input integer target_idx;
    begin
        case (target_idx)
            0: get_target_value = base_threshold;
            1: get_target_value = predicted_iterations;
            2: get_target_value = convergence_mode / 15.0;
            3: get_target_value = aggressiveness / 15.0;
            default: get_target_value = 0;
        endcase
    end
endfunction

// 生成预测
task automatic generate_predictions;
    logic [DATA_WIDTH-1:0] input_features [3:0];
    integer i, j;
    begin
        // 准备输入特征
        for (i = 0; i < 4; i++) begin
            input_features[i] = get_feature_value(i);
        end
        
        // 前向传播
        for (i = 0; i < 4; i++) begin
            model_output[i] = bias_vector[i];
            
            for (j = 0; j < 4; j++) begin
                model_output[i] = model_output[i] + 
                                 weight_matrix[i][j] * input_features[j];
            end
        end
        
        // 生成具体预测
        predicted_iterations <= model_output[1]; // 迭代次数预测
    end
endtask

// 计算置信度
task automatic calculate_confidence;
    logic [DATA_WIDTH-1:0] error_variance;
    logic [DATA_WIDTH-1:0] prediction_stability;
    begin
        // 计算预测误差的方差
        error_variance = calculate_prediction_variance();
        
        // 计算预测稳定性
        prediction_stability = calculate_prediction_stability();
        
        // 综合计算置信度
        confidence_level <= 100 - (error_variance * 10 + prediction_stability * 5);
        
        if (confidence_level > 100) confidence_level <= 100;
        if (confidence_level < 0) confidence_level <= 0;
    end
endtask

// 计算预测方差
function automatic [DATA_WIDTH-1:0] calculate_prediction_variance;
    logic [DATA_WIDTH-1:0] sum, sum_sq, mean, variance;
    integer i, count;
    begin
        sum = 0;
        sum_sq = 0;
        count = 0;
        
        for (i = 0; i < HISTORY_DEPTH; i++) begin
            if (history_buffer[i] != 0) begin
                sum = sum + history_buffer[i];
                sum_sq = sum_sq + history_buffer[i] * history_buffer[i];
                count = count + 1;
            end
        end
        
        if (count > 0) begin
            mean = sum / count;
            variance = (sum_sq / count) - (mean * mean);
        end else begin
            variance = 0;
        end
        
        calculate_prediction_variance = variance;
    end
endfunction

// 计算预测稳定性
function automatic [DATA_WIDTH-1:0] calculate_prediction_stability;
    logic [DATA_WIDTH-1:0] max_diff;
    integer i;
    begin
        max_diff = 0;
        
        for (i = 1; i < HISTORY_DEPTH; i++) begin
            if (history_buffer[i] != 0 && history_buffer[i-1] != 0) begin
                logic [DATA_WIDTH-1:0] diff;
                diff = absolute_difference(history_buffer[i], history_buffer[i-1]);
                
                if (diff > max_diff) begin
                    max_diff = diff;
                end
            end
        end
        
        calculate_prediction_stability = max_diff;
    end
endfunction

// 调整收敛阈值
task automatic adjust_convergence_threshold;
    logic [DATA_WIDTH-1:0] new_threshold;
    begin
        // 基础阈值调整
        new_threshold = base_threshold;
        
        // 基于稀疏度调整
        sparsity_factor = 32'h3F800000 - (matrix_sparsity * 32'h3C23D70A); // 1 - sparsity*0.01
        new_threshold = new_threshold * sparsity_factor;
        
        // 基于矩阵条件数调整
        if (matrix_condition > 7) begin // 病态矩阵
            new_threshold = new_threshold * 32'h3F4CCCCD; // ×0.8
        end
        
        // 基于收敛趋势调整
        case (convergence_trend)
            2'b00: begin // 加速收敛
                new_threshold = new_threshold * 32'h3F666666; // ×0.9
            end
            2'b01: begin // 减速收敛
                new_threshold = new_threshold * 32'h3FA66666; // ×1.3
            end
            2'b10: begin // 振荡
                new_threshold = new_threshold * 32'h3FC00000; // ×1.5
            end
            default: begin // 稳定
                // 不调整
            end
        endcase
        
        // 基于工作负载调整
        workload_factor = 32'h3F800000 + (workload_level * 32'h3C23D70A); // 1 + workload*0.01
        new_threshold = new_threshold * workload_factor;
        
        // 确保阈值在合理范围内 [1e-6, 1e-2]
        if (new_threshold < 32'h358637BD) begin // 1e-6
            new_threshold = 32'h358637BD;
        end else if (new_threshold > 32'h3C23D70A) begin // 1e-2
            new_threshold = 32'h3C23D70A;
        end
        
        threshold_adjusted <= new_threshold;
        adaptive_factor = new_threshold / base_threshold;
    end
endtask

// 调整迭代限制
task automatic adjust_iteration_limits;
    begin
        // 基于预测调整迭代限制模式
        if (predicted_iterations < 20) begin
            iteration_limit_mode <= 3'b001; // 宽松限制
        end else if (predicted_iterations < 50) begin
            iteration_limit_mode <= 3'b010; // 中等限制
        end else begin
            iteration_limit_mode <= 3'b100; // 严格限制
        end
        
        // 基于系统拥塞调整
        if (system_congested) begin
            iteration_limit_mode <= iteration_limit_mode | 3'b001; // 更宽松
        end
        
        // 基于截止时间调整
        if (deadline_approaching) begin
            iteration_limit_mode <= iteration_limit_mode & 3'b110; // 更严格
        end
    end
endtask

// 调整学习参数
task automatic adjust_learning_parameters;
    begin
        // 基于预测置信度调整学习率
        if (confidence_level > 80) begin
            learning_rate <= EWMA_ALPHA * 32'h3F800000; // 正常学习率
        end else if (confidence_level > 50) begin
            learning_rate <= EWMA_ALPHA * 32'h3F4CCCCD; // 较低学习率
        end else begin
            learning_rate <= EWMA_ALPHA * 32'h3ECCCCCD; // 很低学习率
        end
        
        // 调整激进程度
        if (convergence_speed > 50) begin // 收敛快
            aggressiveness <= 4'b0010; // 较低激进度
        end else if (convergence_speed > 20) begin // 中等收敛
            aggressiveness <= 4'b0100; // 中等激进度
        end else begin // 收敛慢
            aggressiveness <= 4'b1000; // 高激进度
        end
        
        // 调整早期停止
        if (ewma_error < base_threshold * 2) begin
            enable_early_stop <= 1;
        end else begin
            enable_early_stop <= 0;
        end
    end
endtask

// 评估调整效果
task automatic evaluate_adjustment_effectiveness;
    logic prediction_correct;
    begin
        // 检查预测是否准确
        if (current_iteration > 0) begin
            logic [7:0] prediction_error;
            
            prediction_error = (current_iteration > predicted_iterations) ?
                              (current_iteration - predicted_iterations) :
                              (predicted_iterations - current_iterations);
            
            prediction_correct = (prediction_error <= 5); // 误差在5次迭代内
            
            total_predictions <= total_predictions + 1;
            
            if (prediction_correct) begin
                success_counter <= success_counter + 1;
                successful_predictions <= success_counter;
            end
            
            // 更新准确率
            if (total_predictions > 0) begin
                current_accuracy <= (success_counter * 100) / total_predictions;
                prediction_accuracy <= current_accuracy;
            end
        end
        
        // 分析收敛趋势
        analyze_convergence_trend();
    end
endtask

// 分析收敛趋势
task automatic analyze_convergence_trend;
    logic [DATA_WIDTH-1:0] error_diff [2:0];
    integer i;
    begin
        // 计算最近误差变化
        for (i = 0; i < 3; i++) begin
            integer idx = (error_ptr - i - 1 + HISTORY_DEPTH) % HISTORY_DEPTH;
            if (i + 1 < 3) begin
                integer prev_idx = (error_ptr - i - 2 + HISTORY_DEPTH) % HISTORY_DEPTH;
                error_diff[i] = error_history[prev_idx] - error_history[idx];
            end
        end
        
        // 判断趋势
        if (error_diff[0] > 0 && error_diff[1] > 0 && error_diff[2] > 0) begin
            convergence_trend <= 2'b00; // 加速收敛
        end else if (error_diff[0] < 0 && error_diff[1] < 0 && error_diff[2] < 0) begin
            convergence_trend <= 2'b01; // 减速收敛
        end else if ((error_diff[0] > 0 && error_diff[1] < 0 && error_diff[2] > 0) ||
                    (error_diff[0] < 0 && error_diff[1] > 0 && error_diff[2] < 0)) begin
            convergence_trend <= 2'b10; // 振荡
        end else begin
            convergence_trend <= 2'b11; // 稳定
        end
        
        // 计算收敛速度
        if (error_history[(error_ptr-1+HISTORY_DEPTH)%HISTORY_DEPTH] > 0) begin
            convergence_speed <= (error_history[(error_ptr-2+HISTORY_DEPTH)%HISTORY_DEPTH] * 100) /
                                error_history[(error_ptr-1+HISTORY_DEPTH)%HISTORY_DEPTH];
        end
    end
endtask

// 更新成功率指标
task automatic update_success_metrics;
    begin
        if (total_predictions > 0) begin
            successful_predictions <= success_counter;
            prediction_accuracy <= (success_counter * 100) / total_predictions;
        end
    end
endtask

// 更新统计信息
task automatic update_statistics;
    begin
        total_adjustments <= total_adjustments + 1;
        
        if (cycle_counter > 0) begin
            adaptation_rate <= (adaptation_counter * 100) / (cycle_counter >> 8);
        end
    end
endtask

// 自适应调整学习率
task automatic adapt_learning_rate;
    begin
        if (current_accuracy > 80) begin
            // 高准确率，增加学习率
            learning_rate <= learning_rate * 32'h3FA66666; // ×1.3
        end else if (current_accuracy < 50) begin
            // 低准确率，减少学习率
            learning_rate <= learning_rate * 32'h3F333333; // ×0.7
        end
        
        // 确保学习率在合理范围内
        if (learning_rate > EWMA_ALPHA * 2) begin
            learning_rate <= EWMA_ALPHA * 2;
        end else if (learning_rate < EWMA_ALPHA / 4) begin
            learning_rate <= EWMA_ALPHA / 4;
        end
    end
endtask

// 辅助函数：计算绝对差
function automatic [DATA_WIDTH-1:0] absolute_difference;
    input [DATA_WIDTH-1:0] a;
    input [DATA_WIDTH-1:0] b;
    logic [DATA_WIDTH-1:0] diff;
    begin
        diff = (a > b) ? (a - b) : (b - a);
        absolute_difference = diff;
    end
endfunction

// 获取当前收敛模式
function automatic [3:0] get_current_convergence_mode;
    begin
        if (convergence_speed > 70) begin
            get_current_convergence_mode = 4'b0001; // 快速模式
        end else if (convergence_speed > 30) begin
            get_current_convergence_mode = 4'b0010; // 标准模式
        end else begin
            get_current_convergence_mode = 4'b0100; // 精确模式
        end
    end
endfunction

// 检查是否需要重置模型
function automatic logic needs_model_reset;
    begin
        needs_model_reset = (current_accuracy < 30 && total_predictions > 50) ||
                           (adaptation_counter > 1000 && current_accuracy < 50);
    end
endfunction

// 重置学习模型
task automatic reset_learning_model;
    integer i, j;
    begin
        for (i = 0; i < 4; i++) begin
            for (j = 0; j < 4; j++) begin
                weight_matrix[i][j] <= (i == j) ? 32'h3F800000 : 0;
            end
            bias_vector[i] <= 0;
        end
        
        learning_rate <= EWMA_ALPHA;
        adaptation_counter <= 0;
    end
endtask

endmodule