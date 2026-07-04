
`include "define.v"

module interrupt_controller #(
    parameter ADR_WIDTH = 6,
    parameter IRQ_LINES = 16 // Default number of interrupt lines
)(
    // --- System Signals ---
    input  wire                 clk,         // System clock (100 MHz)
    input  wire                 rst,         // Synchronous reset (Active-High)

    // --- CPU Bus Interface (FSMC) ---
    input  wire [ADR_WIDTH-1:0] cpu_addr,    // Register address
    input  wire [15:0]          cpu_di,      // Data from STM32 to FPGA
    input  wire                 cpu_wr,      // Write strobe (Active-High)
    input  wire                 cpu_rd,      // Read strobe (Active-High)
    output wire [15:0]          cpu_do,      // Data from FPGA to STM32

    // --- Hardware Interrupt Lines ---
    input  wire [IRQ_LINES-1:0] irq_inputs,  // Raw signals from internal FPGA blocks
    output wire                 irq_out_N    // Global interrupt signal for STM32 (Active-Low)
);

    // =========================================================================
    // 1. REGISTER MAP
    // =========================================================================
    localparam BASE_ADDR      = 6'h00;
    localparam P_OFF_PENDING  = 0; // [RW] Pending interrupts. Clear by writing 1 (W1C).
    localparam P_OFF_MASK     = 1; // [RW] Mask. 1 = Interrupt enabled, 0 = Disabled.
    localparam P_OFF_EDGE_SEL = 2; // [RW] Edge Select. 0 = Rising edge, 1 = Falling edge.
    localparam P_OFF_CTRL     = 3; // [RW] Control. Bit 0: Global interrupt enable.

    // Internal state registers
    reg [IRQ_LINES-1:0] pending_reg;
    reg [IRQ_LINES-1:0] mask_reg;
    reg [IRQ_LINES-1:0] edge_sel_reg;
    reg [15:0]          ctrl_reg;

    // =========================================================================
    // 2. SYNCHRONIZERS & EDGE DETECTORS
    // =========================================================================
    // Three-stage shift register for metastability protection of asynchronous signals
    reg [IRQ_LINES-1:0] sync1_reg, sync2_reg, sync3_reg;

    always @(posedge clk) begin
        if (rst) begin
            sync1_reg <= {IRQ_LINES{1'b0}};
            sync2_reg <= {IRQ_LINES{1'b0}};
            sync3_reg <= {IRQ_LINES{1'b0}};
        end else begin
            sync1_reg <= irq_inputs;
            sync2_reg <= sync1_reg;
            sync3_reg <= sync2_reg;
        end
    end

    // Edge detectors (generates a pulse exactly 1 clock cycle wide)
    wire [IRQ_LINES-1:0] irq_events;
    genvar i;
    generate
        for (i = 0; i < IRQ_LINES; i = i + 1) begin : edge_detectors
            assign irq_events[i] = edge_sel_reg[i] ? 
                                   (sync3_reg[i] & ~sync2_reg[i]) : 
                                   (~sync3_reg[i] & sync2_reg[i]);
        end
    endgenerate

    // =========================================================================
    // 3. BUS INTERFACE DECODING (Symmetrical RTL Design)
    // =========================================================================
    // WRITE access flags
    wire wr_pending  = (cpu_addr == (BASE_ADDR + P_OFF_PENDING))  && cpu_wr;
    wire wr_mask     = (cpu_addr == (BASE_ADDR + P_OFF_MASK))     && cpu_wr;
    wire wr_edge_sel = (cpu_addr == (BASE_ADDR + P_OFF_EDGE_SEL)) && cpu_wr;
    wire wr_ctrl     = (cpu_addr == (BASE_ADDR + P_OFF_CTRL))     && cpu_wr;

    // READ access flags
    wire rd_pending  = (cpu_addr == (BASE_ADDR + P_OFF_PENDING))  && cpu_rd;
    wire rd_mask     = (cpu_addr == (BASE_ADDR + P_OFF_MASK))     && cpu_rd;
    wire rd_edge_sel = (cpu_addr == (BASE_ADDR + P_OFF_EDGE_SEL)) && cpu_rd;
    wire rd_ctrl     = (cpu_addr == (BASE_ADDR + P_OFF_CTRL))     && cpu_rd;

    // =========================================================================
    // 4. MAIN CONTROL LOGIC (Industrial W1C Standard)
    // =========================================================================
    always @(posedge clk) begin
        if (rst) begin
            pending_reg  <= {IRQ_LINES{1'b0}};
            mask_reg     <= {IRQ_LINES{1'b0}};
            edge_sel_reg <= {IRQ_LINES{1'b0}};
            ctrl_reg     <= 16'h0000; 
        end else begin
            
            // --- Write-1-to-Clear (W1C) IMPLEMENTATION ---
            if (wr_pending) begin
                pending_reg <= (pending_reg & ~cpu_di[IRQ_LINES-1:0]) | irq_events;
            end else begin
                pending_reg <= pending_reg | irq_events;
            end

            // Update configuration registers
            if (wr_mask)     mask_reg     <= cpu_di[IRQ_LINES-1:0];
            if (wr_edge_sel) edge_sel_reg <= cpu_di[IRQ_LINES-1:0];
            if (wr_ctrl)     ctrl_reg     <= cpu_di;
        end
    end

    // =========================================================================
    // 5. FSMC BUS READ LOGIC (Snapshot + Combinatorial Mux)
    // =========================================================================  
	/*
    reg [IRQ_LINES-1:0] pending_frozen; 
    reg                 rd_pending_q;   

    // Synchronous Snapshot: Freeze pending_reg on the first read clock cycle
    always @(posedge clk) begin
        if (rst) begin
            rd_pending_q   <= 1'b0;
            pending_frozen <= {IRQ_LINES{1'b0}};
        end else begin
            if (rd_pending && !rd_pending_q) begin
                pending_frozen <= pending_reg;
            end
            rd_pending_q <= rd_pending; 
        end
    end
	
	
    // Combinatorial Output Mux
    assign cpu_do = rd_pending  ? { {(16-IRQ_LINES){1'b0}}, pending_frozen } :
                    rd_mask     ? { {(16-IRQ_LINES){1'b0}}, mask_reg }       :
                    rd_edge_sel ? { {(16-IRQ_LINES){1'b0}}, edge_sel_reg }   :
                    rd_ctrl     ? ctrl_reg                                   :
                    16'h0000;
	*/
	/*
	 assign cpu_do = rd_pending  ? { {(16-IRQ_LINES){1'b0}}, pending_reg } 	  :
                     rd_mask     ? { {(16-IRQ_LINES){1'b0}}, mask_reg }       :
                     rd_edge_sel ? { {(16-IRQ_LINES){1'b0}}, edge_sel_reg }   :
                     rd_ctrl     ? ctrl_reg                                   :
                    16'h0000;		 
	*/				
	// Combinatorial Output Mux: Ïðè ÷òåíèè ADDR_IC_PENDING íàêëàäûâàåì ìàñêó àïïàðàòíî
    assign cpu_do = rd_pending  ? { {(16-IRQ_LINES){1'b0}}, (pending_reg & mask_reg) } : // <-- ÈÑÏÐÀÂËÅÍÈÅ ÇÄÅÑÜ
                    rd_mask     ? { {(16-IRQ_LINES){1'b0}}, mask_reg }        :
                    rd_edge_sel ? { {(16-IRQ_LINES){1'b0}}, edge_sel_reg }    :
                    rd_ctrl     ? ctrl_reg                                    :
                    16'h0000;				
    // =========================================================================
    // 6. PHYSICAL OUTPUT (Direct logic without stretcher)
    // =========================================================================
    
    // Interrupt is considered active if there is at least one flag enabled by the mask
    wire interrupt_active = |(pending_reg & mask_reg);

    // Physical pin irq_out_N is active-low (pulled to ground).
    // Line goes to '0' in hardware and stays there until STM32 clears flags (W1C).
    // Works only if the Global Enable bit is set (ctrl_reg[0]).
    assign irq_out_N = (ctrl_reg[0] && interrupt_active) ? 1'b0 : 1'b1;

endmodule