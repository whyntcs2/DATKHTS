/* Copyright 2024 Grug Huhler.  License SPDX BSD-2-Clause.

   edge_finder raises pulse for one clock when it sees (by
   sampling) a rising edge on sig_in.
*/

module edge_finder
(
 input wire clk,
 input wire reset_n,
 input wire sig_in,
 output reg pulse
);

   reg      sig_in_prev1;
   reg      sig_in_prev2;
   reg      sig_in_prev3;

   always @(posedge clk or negedge reset_n)
     if (!reset_n) begin
        sig_in_prev1 <= 1'b0;
        sig_in_prev2 <= 1'b0;
        sig_in_prev3 <= 1'b0;
     end
     else begin
        sig_in_prev1 <= sig_in;  /* might be metastable */
        sig_in_prev2 <= sig_in_prev1; /*stable */
        sig_in_prev3 <= sig_in_prev2; /*stable */

        if (sig_in_prev3 == 1'b0 && sig_in_prev2 == 1'b1)
          pulse <= 1'b1;
        else
          pulse <= 1'b0;
     end
   
endmodule
