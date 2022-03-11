module tcd132 (
    clock, 
    reset,
	 tcd_M,
	 tcd_SH,
	 tcd_CCD,
	 adc_req,
	 adc_done
);


input clock;
input reset;
input adc_done;
output adc_req;
output tcd_M;
output tcd_SH;
output tcd_CCD;



/* handle when the ADC signals that is has finished processing */
reg r_adc_done_received = 0;
always @ (posedge adc_done or negedge r_adc_req_active or posedge reset)
begin
    if(reset) begin
		r_adc_done_received = 0;
    end else begin
	 
		if(!r_adc_req_active) begin
			r_adc_done_received = 0;
		end else begin
			r_adc_done_received = 1;
		end
	end
end


/* provide the M clock and divide it for CCD and drive SH */
reg r_CCD = 0;
reg r_SH = 0;
reg r_M = 0;
reg r_adc_req_active = 0;
reg r_adc_req_pulse = 0;
reg [2:0] r_clockdiv = 0;
reg [11:0] r_pixel_count = 0;
reg [7:0] r_adc_timeout = 0;
always @ (posedge clock or posedge reset)
begin
    if (reset) begin
		r_CCD = 0;
		r_SH = 0;
		r_M = 0;
		r_adc_req_active = 0;
		r_adc_req_pulse = 0;
		r_clockdiv = 0;
		r_pixel_count = 0;
		r_adc_timeout = 0;
    end else begin
	 
		r_adc_req_pulse = 0;
	 
		if(!r_adc_req_active || r_adc_done_received) begin
		
			r_M = !r_M;
		
			/* divide master clock down by 4 for CCD clock */
			if(r_clockdiv == 0) begin
				r_adc_req_active = 0;
				r_adc_timeout = 0;
				r_clockdiv = 1;
				r_CCD = !r_CCD;
				r_SH = r_CCD && (r_pixel_count == 12'b0);
			end else if(r_clockdiv < 3) begin
				r_clockdiv = r_clockdiv + 1;
			end else if(r_clockdiv == 3) begin
				r_clockdiv = 0;
				
				r_adc_req_active = 1;
				r_adc_req_pulse = 1;
				
				/* count 1091 CCD *edges* ... roughly... there seems to be a factor 2 issue somwhere..?! */
				if (r_pixel_count > 1091/2) begin
					r_pixel_count = 0;
				end else begin
					r_pixel_count = r_pixel_count + 1;
				end
			end
		end else begin
			r_adc_req_pulse = 0;
			if(r_adc_timeout > 100) begin
				r_adc_req_active = 0;
			end else begin
				r_adc_timeout = r_adc_timeout + 1;
			end
		end
	 end
end

assign tcd_M = r_M;
assign tcd_SH = r_SH;
assign tcd_CCD = r_CCD;
assign adc_req = r_adc_req_pulse;

endmodule

