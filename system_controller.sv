`timescale 1ns/1ps

module system_controller #(
    parameter DATA_WIDTH = 32,
    parameter MATRIX_SIZE = 32,
    parameter NUM_PIPELINES = 16
)(
    input logic clk,
    input logic rst_n,
    
    // 配置接口
    input logic [31:0] config_reg,
    input logic config_valid,
    output logic [31:0] status_reg,
    
    // 组件控制信号
    output logic [3:0] engine_mode,
    output logic [2:0] scheduler_mode,
    output logic [1:0] convergence_mode,
    output logic redundancy_enable,
    output logic stealing_enable,
    
    // 组件状态输入
    input logic engine_busy,
    input logic scheduler_busy,
    input logic convergence_done,
    input logic [7:0] iteration_count,
    input logic [15:0] performance_metrics [15:0],
    
    // 错误处理
    input logic error_detected,
    input logic [3:0] error_code,
    output logic error_ack,
    output logic [3:0] recovery_action,
    
    // 系统控制
    output logic system_reset,
    output logic system_pause,
    output logic system_start,
    input logic system_ready
);

// 系统状态
typedef enum logic [3:0] {
    SYS_IDLE = 4'b0000,
    SYS_CONFIG = 4'b0001,
    SYS_INIT = 4'b0010,
    SYS_RUN = 4'b0011,
    SYS_PAUSE = 4'b0100,
    SYS_RECOVER = 4'b0101,
    SYS_SHUTDOWN = 4'b0110,
    SYS_ERROR = 4'b0111,
    SYS_DIAGNOSE = 4'b1000,
    SYS_TUNE = 4'b1001
} sys_state_t;

sys_state_t current_state, next_state;

// 配置寄存器
logic [31:0] current_config;
logic [31:0] saved_config;

// 控制寄存器
logic [7:0] control_flags;
logic [15:0] watchdog_counter;
logic [7:0] error_history [7:0];
logic [2:0] error_ptr;

// 性能调节
logic [7:0] performance_target;
logic [7:0] current_performance;
logic [2:0] tuning_step;

// 恢复策略
logic [3:0] recovery_counter;
logic [7:0] retry_count;

// 初始化
initial begin
    for (int i = 0; i < 8; i++) begin
        error_history[i] = 0;
    end
end

// 主状态机
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        current_state <= SYS_IDLE;
        next_state <= SYS_IDLE;
        current_config <= 32'h00000001;
        saved_config <= 32'h00000001;
        control_flags <= 8'h00;
        watchdog_counter <= 0;
        error_ptr <= 0;
        performance_target <= 80; // 80% 目标性能
        current_performance <= 0;
        tuning_step <= 0;
        recovery_counter <= 0;
        retry_count <= 0;
        
        // 默认控制信号
        engine_mode <= 4'b0001; // 正常模式
        scheduler_mode <= 3'b010; // 平衡调度
        convergence_mode <= 2'b01; // 自适应收敛
        redundancy_enable <= 1;
        stealing_enable <= 1;
        error_ack <= 0;
        recovery_action <= 0;
        system_reset <= 0;
        system_pause <= 0;
        system_start <= 0;
    end else begin
        current_state <= next_state;
        watchdog_counter <= watchdog_counter + 1;
        
        // 更新当前性能
        current_performance <= performance_metrics[3] / 100; // 系统效率
        
        case (current_state)
            SYS_IDLE: begin
                if (config_valid) begin
                    next_state <= SYS_CONFIG;
                end else if (system_ready) begin
                    next_state <= SYS_INIT;
                end
                
                // 重置控制信号
                system_reset <= 0;
                system_pause <= 0;
                error_ack <= 0;
            end
            
            SYS_CONFIG: begin
                // 应用配置
                current_config <= config_reg;
                saved_config <= config_reg;
                
                // 解析配置位
                parse_configuration(config_reg);
                
                next_state <= SYS_INIT;
            end
            
            SYS_INIT: begin
                // 初始化系统
                system_start <= 1;
                
                if (watchdog_counter > 100) begin
                    next_state <= SYS_RUN;
                    system_start <= 0;
                    watchdog_counter <= 0;
                end
            end
            
            SYS_RUN: begin
                // 正常运行监控
                monitor_system_health();
                
                // 性能调节
                if (tuning_step > 0) begin
                    perform_performance_tuning();
                end
                
                // 检查错误
                if (error_detected) begin
                    next_state <= SYS_ERROR;
                    record_error(error_code);
                end else if (control_flags[0]) begin // 暂停请求
                    next_state <= SYS_PAUSE;
                end else if (watchdog_counter > 1000000) begin // 看门狗超时
                    next_state <= SYS_RECOVER;
                end
                
                // 更新状态寄存器
                update_status_register();
            end
            
            SYS_PAUSE: begin
                system_pause <= 1;
                
                if (control_flags[1]) begin // 继续请求
                    next_state <= SYS_RUN;
                    system_pause <= 0;
                    watchdog_counter <= 0;
                end else if (control_flags[2]) begin // 关闭请求
                    next_state <= SYS_SHUTDOWN;
                end
            end
            
            SYS_ERROR: begin
                // 错误处理
                handle_error(error_code);
                
                if (recovery_counter > 10) begin
                    if (retry_count < 3) begin
                        next_state <= SYS_RECOVER;
                        retry_count <= retry_count + 1;
                    } else begin
                        next_state <= SYS_SHUTDOWN;
                    end
                end else begin
                    recovery_counter <= recovery_counter + 1;
                end
            end
            
            SYS_RECOVER: begin
                // 系统恢复
                perform_system_recovery();
                
                if (recovery_counter > 5) begin
                    next_state <= SYS_INIT;
                    recovery_counter <= 0;
                    error_ack <= 1;
                end else begin
                    recovery_counter <= recovery_counter + 1;
                end
            end
            
            SYS_SHUTDOWN: begin
                // 关闭系统
                engine_mode <= 4'b0000;
                scheduler_mode <= 3'b000;
                convergence_mode <= 2'b00;
                redundancy_enable <= 0;
                stealing_enable <= 0;
                
                next_state <= SYS_IDLE;
            end
            
            SYS_DIAGNOSE: begin
                // 系统诊断
                perform_system_diagnosis();
                
                if (watchdog_counter > 5000) begin
                    next_state <= SYS_RUN;
                    watchdog_counter <= 0;
                end
            end
            
            SYS_TUNE: begin
                // 性能调优
                if (tuning_step < 5) begin
                    tuning_step <= tuning_step + 1;
                    adjust_system_parameters();
                end else begin
                    next_state <= SYS_RUN;
                    tuning_step <= 0;
                end
            end
        endcase
    end
end

// 解析配置
task automatic parse_configuration;
    input [31:0] config;
    begin
        // 位[3:0]: 引擎模式
        engine_mode <= config[3:0];
        
        // 位[6:4]: 调度器模式
        scheduler_mode <= config[6:4];
        
        // 位[8:7]: 收敛模式
        convergence_mode <= config[8:7];
        
        // 位[9]: 冗余使能
        redundancy_enable <= config[9];
        
        // 位[10]: 窃取使能
        stealing_enable <= config[10];
        
        // 位[15:11]: 性能目标
        performance_target <= config[15:11] * 5; // 乘以5得到百分比
        
        // 位[23:16]: 控制标志
        control_flags <= config[23:16];
    end
endtask

// 监控系统健康
task automatic monitor_system_health;
    logic [3:0] health_score;
    begin
        health_score = 0;
        
        // 检查PE利用率
        if (performance_metrics[0] > 80*100) begin // >80%
            health_score = health_score + 1;
        end
        
        // 检查系统负载
        if (performance_metrics[1] > 70*100) begin // >70%
            health_score = health_score + 1;
        end
        
        // 检查空闲比例
        if (performance_metrics[2] < 20*100) begin // <20%
            health_score = health_score + 1;
        end
        
        // 检查收敛速率
        if (performance_metrics[6] < 30) begin // <30%
            health_score = health_score + 1;
        end
        
        // 如果健康评分高，考虑性能调优
        if (health_score >= 3 && current_performance < performance_target) begin
            next_state <= SYS_TUNE;
        }
    end
endtask

// 性能调优
task automatic perform_performance_tuning;
    begin
        case (tuning_step)
            1: begin
                // 提高调度器侵略性
                if (scheduler_mode < 3'b100) begin
                    scheduler_mode <= scheduler_mode + 1;
                end
            end
            
            2: begin
                // 调整收敛阈值
                if (convergence_mode < 2'b10) begin
                    convergence_mode <= convergence_mode + 1;
                end
            end
            
            3: begin
                // 启用更积极的工作窃取
                if (stealing_enable && performance_metrics[8] < 60*100) begin
                    // 窃取成功率低，调整策略
                    control_flags[3] <= 1; // 启用激进窃取
                end
            end
            
            4: begin
                // 调整冗余策略
                if (redundancy_enable && performance_metrics[7] < 30*100) begin
                    // 冗余使用率低，减少冗余
                    control_flags[4] <= 1; // 减少冗余
                end
            end
            
            5: begin
                // 最终调整
                if (current_performance < performance_target) begin
                    // 仍未达到目标，重启调优
                    tuning_step <= 0;
                end
            end
        endcase
    end
endtask

// 调整系统参数
task automatic adjust_system_parameters;
    logic [7:0] performance_gap;
    begin
        performance_gap = performance_target - current_performance;
        
        if (performance_gap > 20) begin
            // 性能差距大，采取激进措施
            engine_mode <= 4'b0011; // 高性能模式
            scheduler_mode <= 3'b100; // 激进调度
            convergence_mode <= 2'b10; // 快速收敛
        end else if (performance_gap > 10) begin
            // 中等性能差距
            engine_mode <= 4'b0010; // 平衡模式
            scheduler_mode <= 3'b010; // 平衡调度
        end else if (performance_gap > 0) begin
            // 小性能差距
            engine_mode <= 4'b0001; // 正常模式
            scheduler_mode <= 3'b001; // 保守调度
        end
    end
endtask

// 记录错误
task automatic record_error;
    input [3:0] err_code;
    begin
        error_history[error_ptr] <= {4'h0, err_code};
        error_ptr <= (error_ptr + 1) % 8;
    end
endtask

// 处理错误
task automatic handle_error;
    input [3:0] err_code;
    begin
        case (err_code)
            4'b0001: begin // 未收敛
                recovery_action <= 4'b0001; // 增加迭代次数
            end
            
            4'b0010: begin // 振荡
                recovery_action <= 4'b0010; // 调整收敛阈值
                convergence_mode <= 2'b00; // 严格收敛
            end
            
            4'b0011: begin // 发散
                recovery_action <= 4'b0011; // 重启计算
                system_reset <= 1;
            end
            
            4'b0100: begin // 数值错误
                recovery_action <= 4'b0100; // 检查输入
                next_state <= SYS_DIAGNOSE;
            end
            
            4'b0101: begin // 队列溢出
                recovery_action <= 4'b0101; // 清空队列
                scheduler_mode <= 3'b000; // 停止调度
            end
            
            4'b0110: begin // 缓冲区满
                recovery_action <= 4'b0110; // 暂停输入
                system_pause <= 1;
            end
            
            default: begin
                recovery_action <= 4'b1111; // 未知错误，重启
                next_state <= SYS_RECOVER;
            end
        endcase
    end
endtask

// 执行系统恢复
task automatic perform_system_recovery;
    begin
        case (recovery_counter)
            0: begin
                // 停止所有活动
                system_pause <= 1;
            end
            
            1: begin
                // 重置引擎
                engine_mode <= 4'b0000;
            end
            
            2: begin
                // 清空队列和缓冲区
                scheduler_mode <= 3'b000;
            end
            
            3: begin
                // 应用恢复动作
                execute_recovery_action();
            end
            
            4: begin
                // 重新配置系统
                parse_configuration(saved_config);
            end
            
            5: begin
                // 重启系统
                system_pause <= 0;
                system_start <= 1;
            end
        endcase
    end
endtask

// 执行恢复动作
task automatic execute_recovery_action;
    begin
        case (recovery_action)
            4'b0001: begin // 增加迭代次数
                control_flags[5] <= 1; // 标志位，由引擎解释
            end
            
            4'b0010: begin // 调整收敛阈值
                convergence_mode <= 2'b01; // 中等阈值
            end
            
            4'b0011: begin // 重启计算
                system_reset <= 1;
            end
            
            4'b0100: begin // 检查输入
                next_state <= SYS_DIAGNOSE;
            end
            
            4'b0101: begin // 清空队列
                scheduler_mode <= 3'b001; // 排空模式
            end
            
            4'b0110: begin // 暂停输入
                system_pause <= 1;
            end
        endcase
    end
endtask

// 执行系统诊断
task automatic perform_system_diagnosis;
    logic [7:0] diagnostic_result;
    begin
        diagnostic_result = 0;
        
        // 检查引擎状态
        if (engine_busy) diagnostic_result[0] <= 1;
        
        // 检查调度器状态
        if (scheduler_busy) diagnostic_result[1] <= 1;
        
        // 检查收敛状态
        if (convergence_done) diagnostic_result[2] <= 1;
        
        // 检查性能指标
        for (int i = 0; i < 8; i++) begin
            if (performance_metrics[i] == 0) diagnostic_result[3] <= 1;
        end
        
        // 记录诊断结果到状态寄存器
        status_reg[31:24] <= diagnostic_result;
    end
endtask

// 更新状态寄存器
task automatic update_status_register;
    begin
        status_reg[0] <= engine_busy;
        status_reg[1] <= scheduler_busy;
        status_reg[2] <= convergence_done;
        status_reg[3] <= error_detected;
        status_reg[7:4] <= error_code;
        status_reg[15:8] <= iteration_count;
        status_reg[23:16] <= current_state;
        status_reg[31:24] <= current_performance;
    end
endtask

// 获取错误历史
function automatic [31:0] get_error_history;
    integer i;
    logic [31:0] history;
    begin
        history = 0;
        for (i = 0; i < 8; i++) begin
            history = history | (error_history[i] << (i*4));
        end
        get_error_history = history;
    end
endfunction

// 计算系统健康评分
function automatic [7:0] calculate_health_score;
    logic [7:0] score;
    begin
        score = 100;
        
        // 扣分项
        if (performance_metrics[0] < 50*100) score = score - 20; // PE利用率低
        if (performance_metrics[2] > 50*100) score = score - 15; // 空闲比例高
        if (performance_metrics[3] < 70*100) score = score - 10; // 系统效率低
        if (error_detected) score = score - 30; // 有错误
        
        // 确保在0-100范围内
        if (score > 100) score = 100;
        if (score < 0) score = 0;
        
        calculate_health_score = score;
    end
endfunction

endmodule