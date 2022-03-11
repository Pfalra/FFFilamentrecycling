module pwm (
    clock, 
    reset, 
    pin
);


input clock;
input reset;
output pin;
reg [15:0] r_counter = 0;
reg [15:0] r_counter_cmp = 1;
reg [15:0] r_counter_max = 1500;

assign pin = (r_counter < r_counter_cmp);


always @ (posedge clock or posedge reset)
begin
    if(reset) begin
		r_counter = 0;
    end else begin
		if(r_counter >= r_counter_max) begin
			r_counter = 0;
		end else begin
			r_counter++;
		end
	end
end

endmodule
