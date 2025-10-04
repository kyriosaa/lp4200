// LP4200 Verilog code if u wanna program for FPGA and stuff

module lp4200_power_management (
    input wire clk,                     
    input wire rst_n,                  
    
    // input ports
    input wire micro_usb_detect,       
    input wire jack_detect,             
    input wire [11:0] micro_usb_voltage, 
    input wire [11:0] jack_voltage,     
    
    // BATT IN port
    input wire [11:0] battery_voltage,  
    input wire [11:0] battery_current,  
    input wire [11:0] battery_temp,     
    
    // SYS OUT port
    input wire [11:0] system_current,   
    output reg sys_out_enable,          
    
    // power path control
    output reg [1:0] input_select,      
    output reg battery_charge_enable,   
    output reg [7:0] charge_pwm,       
    
    // status outputs
    output reg [2:0] charge_state,      
    output reg power_good,             
    output reg battery_good,           
    output reg charging_led,           
    output reg [3:0] led_status,        
    output reg fault                
);

    // power source priority states
    localparam SRC_NONE      = 2'b00;
    localparam SRC_MICRO_USB = 2'b01;
    localparam SRC_JACK      = 2'b10;
    
    // charging states
    localparam CHG_IDLE           = 3'b000;
    localparam CHG_PRECHARGE      = 3'b001;
    localparam CHG_CONST_CURRENT  = 3'b010;
    localparam CHG_CONST_VOLTAGE  = 3'b011;
    localparam CHG_COMPLETE       = 3'b100;
    localparam CHG_FAULT          = 3'b101;
    
    // volt thresholds (12-bit ADC normalized)
    localparam INPUT_VOLTAGE_MIN  = 12'd2458;  // 3V min
    localparam INPUT_VOLTAGE_GOOD = 12'd4096;  // 5V optimal
    localparam BATTERY_MIN        = 12'd2458;  // 3V
    localparam BATTERY_PRECHARGE  = 12'd2867;  // 3.5V
    localparam BATTERY_TARGET     = 12'd3440;  // 4.2V
    localparam BATTERY_LOW        = 12'd2703;  // 3.3V
    
    // current thresholds
    localparam CURRENT_PRECHARGE  = 12'd205;   // 0.1C
    localparam CURRENT_FAST       = 12'd2048;  // 1C
    localparam CURRENT_TAPER      = 12'd102;   // 0.05C
    localparam SYSTEM_MAX         = 12'd4095;  // max sys current
    
    // temp limits
    localparam TEMP_MIN = 12'd1024;  // 0°C
    localparam TEMP_MAX = 12'd3072;  // 45°C
    
    reg [1:0] active_input;
    reg [2:0] state;
    reg [2:0] next_state;
    reg [23:0] state_timer;
    reg [7:0] target_pwm;
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            active_input <= SRC_NONE;
            input_select <= SRC_NONE;
        end
        else begin
            // prioritize the DC jack bcs it usually has higher power
            if (jack_detect && jack_voltage >= INPUT_VOLTAGE_GOOD) begin
                active_input <= SRC_JACK;
                input_select <= SRC_JACK;
            end
            else if (micro_usb_detect && micro_usb_voltage >= INPUT_VOLTAGE_GOOD) begin
                active_input <= SRC_MICRO_USB;
                input_select <= SRC_MICRO_USB;
            end
            else begin
                active_input <= SRC_NONE;
                input_select <= SRC_NONE;
            end
        end
    end
    
    always @(*) begin
        power_good = (active_input != SRC_NONE);
    end
    
    always @(*) begin
        battery_good = (battery_voltage >= BATTERY_MIN) && 
                      (battery_temp >= TEMP_MIN) && 
                      (battery_temp <= TEMP_MAX);
    end
    

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sys_out_enable <= 1'b0;
        end
        else begin
            // enable sys out if either the external power is available or the battery voltage is sufficient and theres nothing wrong w it
            if (power_good || (battery_voltage >= BATTERY_LOW && !fault)) begin
                sys_out_enable <= 1'b1;
            end
            else begin
                sys_out_enable <= 1'b0;
            end
        end
    end
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= CHG_IDLE;
            state_timer <= 24'b0;
            charge_pwm <= 8'b0;
        end
        else begin
            state <= next_state;
            
            if (state != CHG_IDLE && state != CHG_COMPLETE && state != CHG_FAULT) begin
                state_timer <= state_timer + 1;
            end
            else begin
                state_timer <= 24'b0;
            end
            
            if (charge_pwm < target_pwm)
                charge_pwm <= charge_pwm + 1;
            else if (charge_pwm > target_pwm)
                charge_pwm <= charge_pwm - 1;
        end
    end
    
    always @(*) begin
        next_state = state;
        battery_charge_enable = 1'b0;
        target_pwm = 8'b0;
        charge_state = state;
        led_status = 4'b0000;
        fault = 1'b0;
        charging_led = 1'b0;  // OFF
        
        if (battery_temp < TEMP_MIN || battery_temp > TEMP_MAX) begin
            next_state = CHG_FAULT;
        end
        
        case (state)
            CHG_IDLE: begin
                led_status = 4'b0001; // power indicator
                
                if (power_good && battery_good) begin
                    if (battery_voltage < BATTERY_PRECHARGE)
                        next_state = CHG_PRECHARGE;
                    else if (battery_voltage < BATTERY_TARGET)
                        next_state = CHG_CONST_CURRENT;
                    else
                        next_state = CHG_COMPLETE;
                end
                
                if (!battery_good)
                    next_state = CHG_FAULT;
            end
            
            CHG_PRECHARGE: begin
                battery_charge_enable = 1'b1;
                charging_led = 1'b1;  
                led_status = 4'b0011;
                target_pwm = 8'd64;   
                
                if (!power_good)
                    next_state = CHG_IDLE;
                else if (battery_voltage >= BATTERY_PRECHARGE)
                    next_state = CHG_CONST_CURRENT;
                else if (battery_voltage < BATTERY_MIN)
                    next_state = CHG_FAULT;
            end
            
            CHG_CONST_CURRENT: begin
                battery_charge_enable = 1'b1;
                charging_led = 1'b1;  
                led_status = 4'b0111; 
                target_pwm = 8'd204; 
                
                if (!power_good)
                    next_state = CHG_IDLE;
                else if (battery_voltage >= (BATTERY_TARGET - 12'd16))
                    next_state = CHG_CONST_VOLTAGE;
            end
            
            CHG_CONST_VOLTAGE: begin
                battery_charge_enable = 1'b1;
                charging_led = 1'b1;  
                led_status = 4'b1111; 
                
                if (battery_current > CURRENT_TAPER)
                    target_pwm = charge_pwm;
                else
                    next_state = CHG_COMPLETE;
                    
                if (!power_good)
                    next_state = CHG_IDLE;
            end
            
            CHG_COMPLETE: begin
                charging_led = 1'b0;  
                led_status = 4'b1000; 
                target_pwm = 8'b0;
                
                if (battery_voltage < (BATTERY_TARGET - 12'd164))
                    next_state = CHG_IDLE;
            end
            
            CHG_FAULT: begin
                fault = 1'b1;
                led_status = 4'b0101; 
                target_pwm = 8'b0;
                
                if (battery_good && battery_temp >= TEMP_MIN && battery_temp <= TEMP_MAX)
                    next_state = CHG_IDLE;
            end
            
            default: begin
                next_state = CHG_IDLE;
            end
        endcase
    end

endmodule