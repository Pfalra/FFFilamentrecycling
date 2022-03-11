module led_drv (
	 adc_valid,
    adc_value,
	 tcd_SH,
	 led0,
	 led1,
	 led2,
	 led3,
	 led4,
	 led5,
	 led6,
	 led7,
	 dummy_out
);

input [11:0] adc_value;
input adc_valid;
input tcd_SH;
output led0;
output led1;
output led2;
output led3;
output led4;
output led5;
output led6;
output led7;
reg r_led0;
reg r_led1;
reg r_led2;
reg r_led3;
reg r_led4;
reg r_led5;
reg r_led6;
reg r_led7;

reg pixel_state;
reg [11:0] pixel_value;
reg [11:0] pixel_count;
reg [11:0] pixel_count_total;
reg [11:0] pixel_value_max_cur;
reg [11:0] pixel_value_min_cur;
reg [11:0] pixel_value_max_next;
reg [11:0] pixel_value_min_next;
reg [11:0] pixel_value_max;
reg [11:0] pixel_value_min;
reg [15:0] pixel_margin_avg;
reg [11:0] pixel_margin_h;
reg [11:0] pixel_margin_l;
reg [11:0] drop_start;
reg [11:0] drop_end;
reg [11:0] drop_width;
reg [15:0] drop_width_avg;

output [11:0] dummy_out;


always @ (posedge tcd_SH or posedge adc_valid)
begin
	if(tcd_SH) begin
		pixel_count = 0;
		/* reset current frame statistics */
	end else begin
		pixel_count = pixel_count + 1;
		pixel_count_total = pixel_count_total + 1;
		pixel_value = adc_value;
		pixel_value_max_cur = pixel_value_max_next;
		pixel_value_min_cur = pixel_value_min_next;
	end
end


always @ (negedge adc_valid)
begin

	if(pixel_count == 0) begin
	
		drop_start = 0;
		drop_end = 0;
		pixel_state = 0;
		pixel_value_max_next = 0;
		drop_width = 0;
		pixel_value_min_next = -1;
		
	end else if(pixel_count > 70 && pixel_count < 1080) begin
	
		/* update current frame statistics */
		if(pixel_value > pixel_value_max_cur) begin
			pixel_value_max_next = pixel_value;
		end
		if(pixel_value < pixel_value_min_cur) begin
			pixel_value_min_next = pixel_value;
		end
		
		if(pixel_value > pixel_margin_h && pixel_state == 0) begin
			pixel_state = 1;
			drop_start = pixel_count;
		end
		if(pixel_value < pixel_margin_l && pixel_state == 1) begin
			pixel_state = 0;
			drop_end = pixel_count;
		end
		
	end else if(pixel_count == 1088) begin
	
		/* update statistics */
		pixel_value_max = pixel_value_max_cur;
		pixel_value_min = pixel_value_min_cur;
		
		pixel_margin_avg = (1 * pixel_margin_avg + (pixel_value_min_cur + (pixel_value_max_cur - pixel_value_min_cur) / 2)) / 2;
		
		if(drop_end - drop_start > drop_width) begin
			drop_width = drop_end - drop_start;
			drop_width_avg = (1 * drop_width_avg + (drop_end - drop_start)) / 2;
		end
		
	end else if(pixel_count == 1089) begin
	
		pixel_margin_h = pixel_margin_avg + 10;
		pixel_margin_l = pixel_margin_avg - 10;
		
	end else if(pixel_count == 1090) begin
	
		r_led7 = drop_width_avg > 132;
		r_led6 = drop_width_avg > 131;
		r_led5 = drop_width_avg > 130;
		r_led4 = drop_width_avg > 129;
		r_led3 = drop_width_avg > 128;
		r_led2 = drop_width_avg > 127;
		r_led1 = drop_width_avg > 126;
		r_led0 = drop_width_avg > 125;
	end 
end

assign led0 = r_led0;
assign led1 = r_led1;
assign led2 = r_led2;
assign led3 = r_led3;
assign led4 = r_led4;
assign led5 = r_led5;
assign led6 = r_led6;
assign led7 = r_led7;
assign dummy_out = drop_width;


endmodule
