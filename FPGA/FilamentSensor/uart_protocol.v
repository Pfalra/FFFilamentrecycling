module uart_protocol (
    clk,
	 sync,
	 adc_valid,
    adc_value,
	 tx_out,
	 tx_valid,
	 tx_busy
);

input clk;
input sync;

input [11:0] adc_value;
input adc_valid;
input tx_busy;
output [7:0] tx_out;
output tx_valid;
reg [7:0] r_tx_out;
reg r_tx_valid;

reg [7:0] state;

assign tx_out = r_tx_out;
assign tx_valid = r_tx_valid && !tx_busy;


always @(posedge adc_valid or posedge tx_busy or posedge sync) begin

	if(sync) begin
		state = 0;
	end else if(tx_busy) begin
		r_tx_valid = 0;
	end else if(adc_valid) begin
		
		case(state)
			0: r_tx_out = "S";
			1: r_tx_out = "Y";
			2: r_tx_out = "N";
			3: r_tx_out = "C";
			4: r_tx_out = adc_value >> 4;
		endcase
		
		if(state < 4) begin
			state = state + 1;
		end
		
		r_tx_valid = 1;
		
	end
end


endmodule
