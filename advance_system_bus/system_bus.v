
/* Advanced System Bus Design : 
		Language - Verilog
		
		properties - Supports upto 4 Masters and 8 Slaves
						 5 stage piplined Arbiter to perform split transaction
						 Masters can hold 4 different priority stages and they allow to change their priority at any time.
						 Two serial buses are used in data transmision
						 master and slave interfaces are similar to the Avalon MM satndard (Avalon Memory Mapped - Altera)
						 Address width and Data width can be easily changed with parameters. (upgrading no of Masters and Slaves can be done with following same pattern used in case statements)
		
		Authors - Bhashitha Mishara, Sandaru Jayawardana, Shan Anjana
		
		License - MIT License		
*/

module system_bus
	#(parameter DATA_WIDTH = 8, 
	parameter ADDR_WIDTH = 14, 
	parameter NO_OF_MASTER = 4, 
	parameter NO_OF_SLAVE = 8,
	parameter [NO_OF_SLAVE*ADDR_WIDTH-1:0] SLAVE_BASE_ADDR = {14'd10, 14'd20, 14'd30, 14'd40, 14'd50, 14'd60, 14'd70, 14'd80}, 
	parameter [NO_OF_SLAVE*ADDR_WIDTH-1:0] SLAVE_END_ADDR = {14'd19, 14'd29, 14'd39, 14'd49, 14'd59, 14'd69, 14'd79, 14'd89}
	) 
	
	(
	input clk,
	input reset_n,
	
	/// MASTER
	input [ADDR_WIDTH*NO_OF_MASTER-1:0]  addr_avm, 
	output [DATA_WIDTH*NO_OF_MASTER-1:0]  readdata_avm,
	input [DATA_WIDTH*NO_OF_MASTER-1:0]  writedata_avm, 
	input [NO_OF_MASTER-1:0]  readen_avm,
	input [NO_OF_MASTER-1:0]  writeen_avm,
	output [NO_OF_MASTER-1:0]  readvalid_avm,
	output [NO_OF_MASTER-1:0]  writevalid_avm,
	input [NO_OF_MASTER-1:0]  waitrequest_avm,

	// priority 
	input [4*NO_OF_MASTER-1:0]priority_no,
	
	/// SALVE
	output [ADDR_WIDTH*NO_OF_SLAVE-1:0]  addr_avs,
	input [DATA_WIDTH*NO_OF_SLAVE-1:0]  readdata_avs,
	output [DATA_WIDTH*NO_OF_SLAVE-1:0]  writedata_avs,
	output [NO_OF_SLAVE-1:0]  readen_avs,
	output [NO_OF_SLAVE-1:0]  writeen_avs,
	input [NO_OF_SLAVE-1:0]  readvalid_avs,
	input [NO_OF_SLAVE-1:0]  writevalid_avs,
	output reg [NO_OF_SLAVE-1:0]  waitrequest_avs
	);
	
	
	reg [NO_OF_MASTER-1:0] readen_avm_last, writeen_avm_last, readen_avm_reg, writeen_avm_reg;
	reg [NO_OF_MASTER-1:0]  readvalid_avm_reg, readvalid_avm_reg_out, writevalid_avm_reg_out;
	reg [DATA_WIDTH-1:0]  readdata_avm_reg [NO_OF_MASTER-1:0];
	reg [7:0] readvalid_avs_last, writevalid_avs_last;
	reg [DATA_WIDTH-1:0]  readdata_avs_reg [NO_OF_SLAVE-1:0];
	reg [NO_OF_SLAVE:0] readvalid_avs_reg=9'b0;
	reg [NO_OF_SLAVE:0] writevalid_avs_reg=9'b0;
	reg [ADDR_WIDTH-1:0] addr_avs_reg [NO_OF_SLAVE-1:0];
	reg [DATA_WIDTH-1:0]  writedata_avs_reg [NO_OF_SLAVE-1:0];
	reg [NO_OF_SLAVE-1:0]  readen_avs_reg, writeen_avs_reg, readen_avs_reg_out, writeen_avs_reg_out;
	
	wire [ADDR_WIDTH-1:0] addr_avm_wire [NO_OF_MASTER-1:0];
	wire [DATA_WIDTH-1:0] writedata_avm_wire [NO_OF_MASTER-1:0];
	wire [3:0] priority_wire [NO_OF_MASTER-1:0];
	wire [4*NO_OF_MASTER-1:0] priority_no_with_active;
	
	reg [7:0] free_slaves = 8'hff;
	reg [3:0] last_state_stage_1_2;
	reg [1:0] master_slave_mux_master;
	reg [7:0] slave_master_demux_slave;
	reg [3:0] counter_s_p_master [NO_OF_MASTER-1:0];
	reg [3:0] value_s_p_master [NO_OF_MASTER-1:0];
	reg [7:0] slave_en_flag;
	reg [4:0] s_p_conversion_counter;
	reg [4:0] s_p_conversion_value;
	reg [3:0] s_p_conversion_counter_4;
	reg [3:0] s_p_conversion_value_4;
	reg stage_2_state;
	reg stage_4_state;
	reg [DATA_WIDTH + 2 - 1:0] shift_reg_slave;
	reg [DATA_WIDTH + ADDR_WIDTH + 2-1:0] shift_reg_master;
	reg [3:0] master_enable;
	reg [7:0] last_state_stage_3_4;
	reg [7:0] last_state_stage_3_4_single;
	reg [7:0] slave_active_stage_3;
	wire [31:0] slave_master_priority;
	wire [31:0] slave_master_priority_with_active;
	wire [3:0] master_enable_wire;
	wire m2s_serial_bus,s2m_serial_bus;
	wire [7:0] master_corresponding_slave [3:0];
	wire [3:0] master_active_stage_1;
	
	reg [3:0] mux_demux_slave_corresponding_master [7:0] ;
	reg [1:0] mux_demux_slave_corresponding_master_2bit [7:0] ;
	reg [2:0] master_corresponding_slave_3bit [NO_OF_MASTER-1:0];
	reg [4:0] counter_s_p_slave [NO_OF_SLAVE-1:0];
	reg [4:0] value_s_p_slave [NO_OF_SLAVE-1:0];
	reg [1:0] state_master_interface [NO_OF_MASTER-1:0];
	reg [1:0] state_slave_interface [NO_OF_SLAVE-1:0];
	reg [7:0] slave_master_mux_slave;
	reg [3:0] slave_master_mux_slave_3b;
	reg [3:0] slave_master_mux_slave_3b_tem;
	reg [3:0] master_releaser_stage;
	reg [7:0] slave_releaser_stage;
	
	assign master_active_stage_1=(readen_avm_reg | writeen_avm_reg) & (~last_state_stage_1_2) & (~{((master_corresponding_slave[3] & free_slaves) == 8'b0),((master_corresponding_slave[2] & free_slaves) == 8'b0),((master_corresponding_slave[1] & free_slaves) == 8'b0),((master_corresponding_slave[0] & free_slaves) == 8'b0)});
	assign slave_master_priority={priority_wire[mux_demux_slave_corresponding_master_2bit[7]],priority_wire[mux_demux_slave_corresponding_master_2bit[6]],priority_wire[mux_demux_slave_corresponding_master_2bit[5]],priority_wire[mux_demux_slave_corresponding_master_2bit[4]],priority_wire[mux_demux_slave_corresponding_master_2bit[3]],priority_wire[mux_demux_slave_corresponding_master_2bit[2]],priority_wire[mux_demux_slave_corresponding_master_2bit[1]],priority_wire[mux_demux_slave_corresponding_master_2bit[0]]};
	assign readen_avs = readen_avs_reg_out;
	assign writeen_avs = writeen_avs_reg_out;
	assign readvalid_avm=readvalid_avm_reg_out;
	assign writevalid_avm=writevalid_avm_reg_out;
	assign master_enable_wire=master_enable;
	assign m2s_serial_bus=shift_reg_master[DATA_WIDTH + ADDR_WIDTH + 1];
	assign s2m_serial_bus=shift_reg_slave[DATA_WIDTH + 1];
	
	//*************************** DATA PRE-PROCESSING, PARALLEL SERIAL CONVERSIONS AT END POINTS *************************//
	
	genvar i, j, k;
	
	generate
		for ( i=0;i<NO_OF_MASTER;i=i+1) // VECTORIZE_INTERFACE_SIGNALS
			begin : VECTORIZE_INTERFACE_SIGNALS
				assign priority_wire[i]=priority_no[i*4+3:i*4];
				assign addr_avm_wire[i]=addr_avm[(i+1)*ADDR_WIDTH-1:i*ADDR_WIDTH];
				assign writedata_avm_wire[i]=writedata_avm[(i+1)*DATA_WIDTH-1:i*DATA_WIDTH];
			end
		for ( k=0;k<NO_OF_MASTER;k=k+1) // ADDRESS_DECODING_0
			begin : ADDRESS_DECODING_0 // to identify the slave devices corresponding to the global address and assign them to respective master
				for ( i=0;i<NO_OF_SLAVE;i=i+1)
					begin : ADDRESS_DECODING_1
						assign master_corresponding_slave[k][i]=(addr_avm[ADDR_WIDTH*k-1+ADDR_WIDTH:ADDR_WIDTH*k] >= SLAVE_BASE_ADDR[(i+1)*ADDR_WIDTH-1:i*ADDR_WIDTH])? ((addr_avm[ADDR_WIDTH*k-1+ADDR_WIDTH:ADDR_WIDTH*k] <= SLAVE_END_ADDR[(i+1)*ADDR_WIDTH-1:i*ADDR_WIDTH])? 1'b1:1'b0):1'b0 ;
					end
			end
		for ( i=0;i<NO_OF_MASTER;i=i+1) // BIT_CONVERSION_8_3
			begin : BIT_CONVERSION_8_3
				always @(*)
					begin
						case(master_corresponding_slave[i])
							8'b00000001:
								begin
									master_corresponding_slave_3bit[i]=3'b000;
								end
							8'b00000010:
								begin
									master_corresponding_slave_3bit[i]=3'b001;
								end
							8'b00000100:
								begin
									master_corresponding_slave_3bit[i]=3'b010;
								end
							8'b00001000:
								begin
									master_corresponding_slave_3bit[i]=3'b011;
								end
							8'b00010000:
								begin
									master_corresponding_slave_3bit[i]=3'b100;
								end
							8'b00100000:
								begin
									master_corresponding_slave_3bit[i]=3'b101;
								end
							8'b01000000:
								begin
									master_corresponding_slave_3bit[i]=3'b110;
								end
							8'b10000000:
								begin
									master_corresponding_slave_3bit[i]=3'b111;
								end
							default:
								begin
									master_corresponding_slave_3bit[i]=3'b00;
								end
						endcase
					end
			end
			
		for ( i=0;i<NO_OF_MASTER;i=i+1) // IDENTIFY_MASTER_REQUEST_DECTING_WITH_EDGE
			begin : IDENTIFY_MASTER_REQUEST_DECTING_WITH_EDGE
				always @(posedge clk)
					begin
						case({((~readen_avm_last[i]) & readen_avm[i]), master_releaser_stage[i]})
							2'b10,2'b11:
								begin
									readen_avm_reg[i]<=1'b1;
								end
							2'b01:
								begin
									readen_avm_reg[i]<=1'b0;
								end
							2'b00:
								begin
									readen_avm_reg[i]<=readen_avm_reg[i];
								end
							default:
								begin
									readen_avm_reg[i]<=1'b0;
								end
						endcase
						
						case({((~writeen_avm_last[i]) & writeen_avm[i]), master_releaser_stage[i]})
							2'b10,2'b11:
								begin
									writeen_avm_reg[i]<=1'b1;
								end
							2'b01:
								begin
									writeen_avm_reg[i]<=1'b0;
								end
							2'b00:
								begin
									writeen_avm_reg[i]<=writeen_avm_reg[i];
								end
							default:
								begin
									writeen_avm_reg[i]<=1'b0;
								end
						endcase
					end
			end
			
		for ( i=0;i<NO_OF_SLAVE;i=i+1) // BIT_CONVERSION_4_2
			begin : BIT_CONVERSION_4_2
				always @(*)
					begin
						case(mux_demux_slave_corresponding_master[i])
							4'b0001:
								begin
									mux_demux_slave_corresponding_master_2bit[i]=2'b00;
								end
							4'b0010:
								begin
									mux_demux_slave_corresponding_master_2bit[i]=2'b01;
								end
							4'b0100:
								begin
									mux_demux_slave_corresponding_master_2bit[i]=2'b10;
								end
							4'b1000:
								begin
									mux_demux_slave_corresponding_master_2bit[i]=2'b11;
								end
							default:
								begin
									mux_demux_slave_corresponding_master_2bit[i]=2'b00;
								end
						endcase
					end
			end
		for ( i=0;i<NO_OF_MASTER;i=i+1) // DETECT_MASTER_REQUESTS_WITH_PRIORITY
			begin : DETECT_MASTER_REQUESTS_WITH_PRIORITY
				assign priority_no_with_active[i*4]=priority_no[i*4] & master_active_stage_1[i];
				assign priority_no_with_active[i*4+1]=priority_no[i*4+1] & master_active_stage_1[i];
				assign priority_no_with_active[i*4+2]=priority_no[i*4+2] & master_active_stage_1[i];
				assign priority_no_with_active[i*4+3]=priority_no[i*4+3] & master_active_stage_1[i];
			end
			
		for ( i=0;i<NO_OF_SLAVE;i=i+1) // DETECT_SLAVE_READY_WITH_PRIORITY
			begin : DETECT_SLAVE_READY_WITH_PRIORITY
				assign slave_master_priority_with_active[i*4]=slave_master_priority[i*4] & slave_active_stage_3[i];
				assign slave_master_priority_with_active[i*4+1]=slave_master_priority[i*4+1] & slave_active_stage_3[i];
				assign slave_master_priority_with_active[i*4+2]=slave_master_priority[i*4+2] & slave_active_stage_3[i];
				assign slave_master_priority_with_active[i*4+3]=slave_master_priority[i*4+3] & slave_active_stage_3[i];
			end
			
		for (i=0;i<NO_OF_SLAVE;i=i+1) // SAMPLE_DATA_FROM_SLAVE_INTERFACE
			begin : SAMPLE_DATA_FROM_SLAVE_INTERFACE
				assign writedata_avs[(i+1)*DATA_WIDTH-1:i*DATA_WIDTH] = writedata_avs_reg[i];
				assign addr_avs[(i+1)*ADDR_WIDTH-1:i*ADDR_WIDTH]=addr_avs_reg[i]-SLAVE_BASE_ADDR[i];
				always @(posedge clk or negedge reset_n)	
					begin
						if (~reset_n)
							begin
								readvalid_avs_reg[i]<=1'b0;
								writevalid_avs_reg[i]<=1'b0;
								readdata_avs_reg[i]<=8'b0;
								waitrequest_avs[i]<=1'b0;
							end
						else	
							begin
								case ({(readvalid_avs[i] & (~readvalid_avs_last[i])),(writevalid_avs[i] & (~writevalid_avs_last[i]))})
									2'b11:
										begin
											readvalid_avs_reg[i]<=readvalid_avs[i];
											writevalid_avs_reg[i]<=writevalid_avs[i];
											readdata_avs_reg[i]<=readdata_avs[i*8+7:i*8];
											waitrequest_avs[i]<=1'b1;
										end
									2'b10:
										begin
											readvalid_avs_reg[i]<=readvalid_avs[i];
											writevalid_avs_reg[i]<=writevalid_avs[i];
											readdata_avs_reg[i]<=readdata_avs[i*8+7:i*8];
											waitrequest_avs[i]<=1'b1;
										end
									2'b01:
										begin
											readvalid_avs_reg[i]<=readvalid_avs[i];
											writevalid_avs_reg[i]<=writevalid_avs[i];
											readdata_avs_reg[i]<=readdata_avs_reg[i];
											waitrequest_avs[i]<=1'b1;
										end
									2'b00:
										begin
											readvalid_avs_reg[i]<=readvalid_avs_reg[i];
											writevalid_avs_reg[i]<=writevalid_avs_reg[i];
											readdata_avs_reg[i]<=readdata_avs_reg[i];
											waitrequest_avs[i]<=1'b0;
										end
									default:
										begin
											readvalid_avs_reg[i]<=1'b0;
											writevalid_avs_reg[i]<=1'b0;
											readdata_avs_reg[i]<=8'b0;
											waitrequest_avs[i]<=1'b0;
										end
								endcase
							end
					end
			end
			
		for (j=0;j<NO_OF_SLAVE;j=j+1) // SERIAL_TO_PARALLEL_CONVERSION_AT_SLAVE_INTERFACES
			begin : SERIAL_TO_PARALLEL_CONVERSION_AT_SLAVE_INTERFACES
				always @(posedge clk or negedge reset_n) 
					begin
						if(~reset_n)
							begin
								counter_s_p_slave[j]<=5'b0;
								value_s_p_slave[j]<=5'b0;	
								state_slave_interface[j]<=2'b0;
								writeen_avs_reg[j] <= 1'b0;
								readen_avs_reg[j] <= 1'b0;
								addr_avs_reg[j]<={ADDR_WIDTH{1'b0}};
								writedata_avs_reg[j]<=8'b0;
								writeen_avs_reg_out[j]<=1'b0;
								readen_avs_reg_out[j] <=1'b0;
							end
						else	
							begin
								case(state_slave_interface[j])
									2'b0:
										begin
											writeen_avs_reg[j] <= writeen_avs_reg[j];
											counter_s_p_slave[j]<=5'b0;
											writedata_avs_reg[j]<=writedata_avs_reg[j];
											addr_avs_reg[j]<=addr_avs_reg[j];
											value_s_p_slave[j]<=value_s_p_slave[j];
											if (slave_en_flag[j])
												begin
													state_slave_interface[j] <=2'b1;
													readen_avs_reg[j] <= m2s_serial_bus;
												end
											else
												begin
													state_slave_interface[j] <=2'b0;
													readen_avs_reg[j] <= 1'b0;
												end
											if (writeen_avs_reg_out[j] & writevalid_avs[j])
												begin
													writeen_avs_reg_out[j]<=1'b0;
												end
											else
												begin
													writeen_avs_reg_out[j]<=writeen_avs_reg_out[j];
												end
											if (readen_avs_reg_out[j] & readvalid_avs[j])
												begin
													readen_avs_reg_out[j] <=1'b0;
												end
											else
												begin
													readen_avs_reg_out[j] <=readen_avs_reg_out[j];
												end		
										end
									2'b01:
										begin
											writeen_avs_reg[j] <= m2s_serial_bus;
											readen_avs_reg[j] <= readen_avs_reg[j];			
											counter_s_p_slave[j]<=5'b0;
											state_slave_interface[j] <=2'b10;			
											writedata_avs_reg[j]<=writedata_avs_reg[j];
											addr_avs_reg[j]<=addr_avs_reg[j];			
											if(readen_avs_reg[j])
												begin
													value_s_p_slave[j]<=ADDR_WIDTH-1'b1;
												end
											else
												begin
													value_s_p_slave[j]<=ADDR_WIDTH+DATA_WIDTH-1'b1;
												end
											if (writeen_avs_reg_out[j] & writevalid_avs[j])
												begin
													writeen_avs_reg_out[j]<=1'b0;
												end
											else
												begin
													writeen_avs_reg_out[j]<=writeen_avs_reg_out[j];
												end
											if (readen_avs_reg_out[j] & readvalid_avs[j])
												begin
													readen_avs_reg_out[j] <=1'b0;
												end
											else
												begin
													readen_avs_reg_out[j] <=readen_avs_reg_out[j];
												end
										end
									2'b10:
										begin
											writeen_avs_reg[j] <= writeen_avs_reg[j];
											readen_avs_reg[j] <= readen_avs_reg[j];
											value_s_p_slave[j]<=value_s_p_slave[j];
											writedata_avs_reg[j]<={writedata_avs_reg[j],addr_avs_reg[j][ADDR_WIDTH-1]};			
											addr_avs_reg[j]<={addr_avs_reg[j],m2s_serial_bus};			
										if(value_s_p_slave[j] == counter_s_p_slave[j] )
											begin
												counter_s_p_slave[j]<=5'd0;
												writeen_avs_reg_out[j]<=writeen_avs_reg[j];
												readen_avs_reg_out[j] <= readen_avs_reg[j];
												state_slave_interface[j] <=2'b0;
											end
										else			
											begin
												counter_s_p_slave[j]<=counter_s_p_slave[j]+5'd1;
												writeen_avs_reg_out[j]<=writeen_avs_reg_out[j];
												readen_avs_reg_out[j] <=readen_avs_reg_out[j];
												state_slave_interface[j] <=2'b10;
											end					
										end
												
									default:
										begin
											counter_s_p_slave[j]<=5'b0;
											value_s_p_slave[j]<=5'b0;	
											state_slave_interface[j]<=2'b0;
											writeen_avs_reg[j] <= 1'b0;
											readen_avs_reg[j] <= 1'b0;
											addr_avs_reg[j]<={ADDR_WIDTH{1'b0}};
											writedata_avs_reg[j]<=8'b0;
											writeen_avs_reg_out[j]<=1'b0;
											readen_avs_reg_out[j] <=1'b0;
										end		
								endcase
							end
					end
		end
		
		for (j=0;j<NO_OF_MASTER;j=j+1) 
			begin : SERIAL_TO_PARALLEL_CONVERSION_AT_MASTER_INTERFACES
				assign readdata_avm[(j+1)*DATA_WIDTH-1:j*DATA_WIDTH]=readdata_avm_reg[j];
				always @(posedge clk or negedge reset_n) 
					begin
						if(~reset_n)
							begin
								
								counter_s_p_master[j]<=4'b0;
								value_s_p_master[j]<=4'b0;	
								state_master_interface[j]<=2'b0;
								readdata_avm_reg[j] <= 8'b0;
								readvalid_avm_reg[j] <= 1'b0;
								readvalid_avm_reg_out[j] <= 1'b0;
								writevalid_avm_reg_out[j] <= 1'b0;
								
							end
						else	
							begin
								case(state_master_interface[j])
									2'b0:
										begin
											counter_s_p_master[j]<=4'b0;
											value_s_p_master[j]<=value_s_p_master[j];
											readdata_avm_reg[j] <= readdata_avm_reg[j];
											if (master_enable_wire[j])
												begin
													state_master_interface[j] <=2'b1;
												end
											else
												begin
													state_master_interface[j] <=2'b0;
												end
											casex({master_enable_wire[j],(readen_avm[j] | writeen_avm[j] | waitrequest_avm[j])})
												2'b1x:
													begin
														readvalid_avm_reg[j] <= s2m_serial_bus;
														readvalid_avm_reg_out[j] <= 1'b0;
														writevalid_avm_reg_out[j] <= 1'b0;
													end
												2'b01:
													begin
														readvalid_avm_reg[j] <= readvalid_avm_reg[j];
														readvalid_avm_reg_out[j] <= readvalid_avm_reg_out[j];
														writevalid_avm_reg_out[j] <= writevalid_avm_reg_out[j];
													end
												2'b00:
													begin
														readvalid_avm_reg[j] <= 1'b0;
														readvalid_avm_reg_out[j] <= 1'b0;
														writevalid_avm_reg_out[j] <= 1'b0;
													end
												default:
													begin
														readvalid_avm_reg[j] <= 1'b0;
														readvalid_avm_reg_out[j] <= 1'b0;
														writevalid_avm_reg_out[j] <= 1'b0;
													end
											endcase
										end
									2'b01:
										begin
											counter_s_p_master[j]<=4'b0;
											readvalid_avm_reg[j] <= readvalid_avm_reg[j];
											
											if(readvalid_avm_reg[j])
												begin
													value_s_p_master[j]<=DATA_WIDTH-1'b1;
													state_master_interface[j] <=2'b10;
													writevalid_avm_reg_out[j] <= s2m_serial_bus;
												end
											else													
												begin
													value_s_p_master[j]<=4'd0;
													state_master_interface[j] <=2'b0;
													writevalid_avm_reg_out[j] <= s2m_serial_bus;
												end
											readvalid_avm_reg_out[j] <= 1'b0;
											readdata_avm_reg[j] <= readdata_avm_reg[j];
										end
									2'b10:
										begin
											value_s_p_master[j]<=value_s_p_master[j];
											readvalid_avm_reg[j] <= readvalid_avm_reg[j];
											readdata_avm_reg[j] <= {readdata_avm_reg[j],s2m_serial_bus};//shift_reg_slave[9]};
											writevalid_avm_reg_out[j] <= 1'b0;
											if(value_s_p_master[j] == counter_s_p_master[j] )
												begin
													counter_s_p_master[j]<=4'd0;
													readvalid_avm_reg_out[j] <= readvalid_avm_reg[j];
													state_master_interface[j] <=2'b0;
												end
											else
											
												begin
													counter_s_p_master[j]<=counter_s_p_master[j]+4'd1;
													readvalid_avm_reg_out[j] <= 1'b0;
													state_master_interface[j] <=2'b10;
												end	
										end
									default:
										begin
											readdata_avm_reg[j] <= 8'b0;
											counter_s_p_master[j]<=4'b0;
											value_s_p_master[j]<=4'b0;	
											state_master_interface[j]<=2'b0;
											readvalid_avm_reg[j] <= 1'b0;
											readvalid_avm_reg_out[j] <= 1'b0;
											writevalid_avm_reg_out[j] <= 1'b0;
										end
								endcase
							end
					end
		end
	endgenerate
	
	always @(posedge clk or negedge reset_n)
		begin
			if(~reset_n)
				begin
					readvalid_avs_last<=8'b0;
					writevalid_avs_last<=8'b0;
					readen_avm_last<=8'b0;
					writeen_avm_last<=8'b0;
				end
			else
				begin
					readvalid_avs_last<=readvalid_avs;
					writevalid_avs_last<=writevalid_avs;
					readen_avm_last<=readen_avm;
					writeen_avm_last<=writeen_avm;
				end
		end
	
	//*************************** Pipeline stages of ARBITER ***************************//
	/*
	 1. Master identify and choose (based of valid signals and priority) and Identify the slave
	 2. write data to corresponding slave (interface registers) (Bus acquire)
	 3. Identify replied slaves.
	 4. Write the reply to corresponding master (Data/ack) (Bus acquire)
	 5. Releaser 
	*/
	//*************************** PIPELINE STAGE 1: (SELECT MASTER PORT AND ASSIGN CONTROL SIGNALS TO MUX AND DEMUX) *************************//
	
	always @(posedge clk or negedge reset_n) 
		begin
			if (~reset_n)
				begin
					master_slave_mux_master <=2'b0;
					slave_master_demux_slave<=8'b0;
					mux_demux_slave_corresponding_master[0] <= 4'b0;
					mux_demux_slave_corresponding_master[1] <= 4'b0;
					mux_demux_slave_corresponding_master[2] <= 4'b0;
					mux_demux_slave_corresponding_master[3] <= 4'b0;
					mux_demux_slave_corresponding_master[4] <= 4'b0;
					mux_demux_slave_corresponding_master[5] <= 4'b0;
					mux_demux_slave_corresponding_master[6] <= 4'b0;
					mux_demux_slave_corresponding_master[7] <= 4'b0;
				end
			else
				begin      
					casex({priority_no_with_active}) 
						16'b1000xxxxxxxxxxxx:
							begin
								master_slave_mux_master <=2'b11;
								slave_master_demux_slave<=master_corresponding_slave[3];
								mux_demux_slave_corresponding_master[master_corresponding_slave_3bit[3]] <= 4'b1000;//2'b11;
							end
						16'b0xxx1000xxxxxxxx:
							begin
								master_slave_mux_master <=2'b10;
								slave_master_demux_slave<=master_corresponding_slave[2];
								mux_demux_slave_corresponding_master[master_corresponding_slave_3bit[2]] <= 4'b0100;
							end
						16'b0xxx0xxx1000xxxx:
							begin
								master_slave_mux_master <=2'b01;
								slave_master_demux_slave<=master_corresponding_slave[1];
								mux_demux_slave_corresponding_master[master_corresponding_slave_3bit[1]] <= 4'b0010;
							end
						16'b0xxx0xxx0xxx1000:
							begin
								master_slave_mux_master <=2'b0;
								slave_master_demux_slave<=master_corresponding_slave[0];
								mux_demux_slave_corresponding_master[master_corresponding_slave_3bit[0]] <= 4'b0001;
							end
						16'b01000xxx0xxx0xxx:
							begin
								master_slave_mux_master <=2'b11;
								slave_master_demux_slave<=master_corresponding_slave[3];
								mux_demux_slave_corresponding_master[master_corresponding_slave_3bit[3]] <= 4'b1000;
							end
						16'b00xx01000xxx0xxx:
							begin
								master_slave_mux_master <=2'b10;
								slave_master_demux_slave<=master_corresponding_slave[2];
								mux_demux_slave_corresponding_master[master_corresponding_slave_3bit[2]] <= 4'b0100;
							end
						16'b00xx00xx01000xxx:
							begin
								master_slave_mux_master <=2'b01;
								slave_master_demux_slave<=master_corresponding_slave[1];
								mux_demux_slave_corresponding_master[master_corresponding_slave_3bit[1]] <= 4'b0010;
							end
							
						16'b00xx00xx00xx0100:
							begin
								master_slave_mux_master <=2'b0;
								slave_master_demux_slave<=master_corresponding_slave[0];
								mux_demux_slave_corresponding_master[master_corresponding_slave_3bit[0]] <= 4'b0001;
							end
						16'b001000xx00xx00xx:
							begin
								master_slave_mux_master <=2'b11;
								slave_master_demux_slave<=master_corresponding_slave[3];
								mux_demux_slave_corresponding_master[master_corresponding_slave_3bit[3]] <= 4'b1000;
							end
						16'b000x001000xx00xx:
							begin
								master_slave_mux_master <=2'b10;
								slave_master_demux_slave<=master_corresponding_slave[2];
								mux_demux_slave_corresponding_master[master_corresponding_slave_3bit[2]] <= 4'b0100;
							end
						16'b000x000x001000xx:
							begin
								master_slave_mux_master <=2'b01;
								slave_master_demux_slave<=master_corresponding_slave[1];
								mux_demux_slave_corresponding_master[master_corresponding_slave_3bit[1]] <= 4'b0010;
							end
						16'b000x000x000x0010:
							begin
								master_slave_mux_master <=2'b0;
								slave_master_demux_slave<=master_corresponding_slave[0];
								mux_demux_slave_corresponding_master[master_corresponding_slave_3bit[0]] <= 4'b0001;
							end
						16'b0001000x000x000x:
							begin
								master_slave_mux_master <=2'b11;
								slave_master_demux_slave<=master_corresponding_slave[3];
								mux_demux_slave_corresponding_master[master_corresponding_slave_3bit[3]] <= 4'b1000;
							end
						16'b00000001000x000x:
							begin
								master_slave_mux_master <=2'b10;
								slave_master_demux_slave<=master_corresponding_slave[2];
								mux_demux_slave_corresponding_master[master_corresponding_slave_3bit[2]] <= 4'b0100;
							end
						16'b000000000001000x:
							begin
								master_slave_mux_master <=2'b01; // mux- 01,  demux - 00000100
								slave_master_demux_slave<=master_corresponding_slave[1];
								mux_demux_slave_corresponding_master[master_corresponding_slave_3bit[1]] <= 4'b0010;
							end
						16'b0000000000000001:
							begin
								master_slave_mux_master <=2'b0;
								slave_master_demux_slave<=master_corresponding_slave[0];
								mux_demux_slave_corresponding_master[master_corresponding_slave_3bit[0]] <= 4'b0001;
							end
						default:
							begin
								master_slave_mux_master <=2'b0;
								slave_master_demux_slave<=8'b0;
							end
					endcase
				end
		
		end
	
	//*************************** PIPELINE STAGE 2: TRANSFER DATA TO SLAVE PORTS WITH PARALLEL TO SERIAL DATA CONVERSION *************************//
	
	always @(posedge clk or negedge reset_n)// stage 2
		begin
			if(~reset_n)
				begin
					slave_en_flag <= 8'b0;
					s_p_conversion_counter <= 5'b0;
					s_p_conversion_value<=5'b0;
					shift_reg_master<={(DATA_WIDTH+ADDR_WIDTH+2){1'b0}};
					last_state_stage_1_2<=4'b0;
					stage_2_state<=1'b0;
					free_slaves<=8'hff;
				end
			else
				begin
					case(stage_2_state)
						1'b0: // writedata_avm_wire
							begin
								s_p_conversion_counter <= 5'b0;
								if (writeen_avm[master_slave_mux_master])
									begin
										s_p_conversion_value <= DATA_WIDTH+ADDR_WIDTH+1'b1; //
										                               // 0                               1                                   8'b0                                      // 45              // 01
										shift_reg_master <= {readen_avm[master_slave_mux_master],writeen_avm[master_slave_mux_master] ,writedata_avm_wire[master_slave_mux_master], addr_avm_wire[master_slave_mux_master]};
									end
								else
									begin
										s_p_conversion_value <= ADDR_WIDTH+1'b1;
										shift_reg_master <= {readen_avm[master_slave_mux_master],writeen_avm[master_slave_mux_master] , addr_avm_wire[master_slave_mux_master], {DATA_WIDTH{1'b0}}};
									end
									
								if(slave_master_demux_slave!=8'b0) // 00000100
									begin 
										slave_en_flag <= slave_master_demux_slave;
										last_state_stage_1_2 <= (last_state_stage_1_2 ^ master_releaser_stage) | (1'b1 << master_slave_mux_master);
										stage_2_state <= 1'b1;
										free_slaves <= ((free_slaves|slave_releaser_stage) ^ slave_master_demux_slave);
									end
								else
									begin
										slave_en_flag<=8'b0;
										
										stage_2_state <= stage_2_state;
										last_state_stage_1_2 <= (last_state_stage_1_2 ^ master_releaser_stage);
										free_slaves <=(free_slaves|slave_releaser_stage);
									end
							end
						1'b1:
							begin
								s_p_conversion_value<=s_p_conversion_value; //17
								slave_en_flag<=8'b0;
								last_state_stage_1_2 <= (last_state_stage_1_2 ^ master_releaser_stage);
								free_slaves<=(free_slaves|slave_releaser_stage);
								shift_reg_master<= {shift_reg_master[DATA_WIDTH+ADDR_WIDTH:0],1'b0};
								if(s_p_conversion_counter == s_p_conversion_value)
									begin
										stage_2_state <= 1'b0;
										s_p_conversion_counter <= 5'b0;
									end
								else
									begin
										stage_2_state <= stage_2_state;
										s_p_conversion_counter <= s_p_conversion_counter + 5'b1;
									end
							end
						default:
							begin
								slave_en_flag <= 8'b0;
								s_p_conversion_counter <= 5'b0;
								s_p_conversion_value<=5'b0;
								shift_reg_master<={(DATA_WIDTH+ADDR_WIDTH+2){1'b0}};
								last_state_stage_1_2<=4'b0;
								stage_2_state<=1'b0;
								free_slaves<=8'hff;
							end
					endcase
				end
		
		end
		
	always @(posedge clk or negedge reset_n)
		begin 
			if (~reset_n)
				begin
					slave_active_stage_3=8'b0;
				end
			else
				begin
					slave_active_stage_3=(slave_active_stage_3|(((~readvalid_avs_last) & readvalid_avs) |  ((~writevalid_avs_last) & writevalid_avs))) & (~last_state_stage_3_4);
				end
				
		end
	
	//*************************** PIPELINE STAGE3: SELECT READY SLAVE PORTS AND SET CONTROL SIGNALS TO MUX AND DEMUX *************************//
	
	always @(posedge clk or negedge reset_n) 
		begin
			if (~reset_n)
				begin
					slave_master_mux_slave<=8'b0;
					slave_master_mux_slave_3b <= 4'b0;
				end
			else
				begin         // 0001                  00000100 
					casex({slave_master_priority_with_active})
						32'b1000xxxxxxxxxxxxxxxxxxxxxxxxxxxx:
							begin
								slave_master_mux_slave<=8'b10000000;
								slave_master_mux_slave_3b <= 4'b111;
							end
						32'b0xxx1000xxxxxxxxxxxxxxxxxxxxxxxx:
							begin
								slave_master_mux_slave<=8'b01000000;
								slave_master_mux_slave_3b <= 4'b110;
							end
						32'b0xxx0xxx1000xxxxxxxxxxxxxxxxxxxx:
							begin
								slave_master_mux_slave<=8'b00100000;
								slave_master_mux_slave_3b <= 4'b101;
							end
						32'b0xxx0xxx0xxx1000xxxxxxxxxxxxxxxx:
							begin
								slave_master_mux_slave<=8'b00010000;
								slave_master_mux_slave_3b <= 4'b100;
							end
						32'b0xxx0xxx0xxx0xxx1000xxxxxxxxxxxx:
							begin
								slave_master_mux_slave<=8'b00001000;
								slave_master_mux_slave_3b <= 4'b011;
							end
						32'b0xxx0xxx0xxx0xxx0xxx1000xxxxxxxx:
							begin
								slave_master_mux_slave<=8'b00000100;
								slave_master_mux_slave_3b <= 4'b010;
							end
						32'b0xxx0xxx0xxx0xxx0xxx0xxx1000xxxx:
							begin
								slave_master_mux_slave<=8'b00000010;
								slave_master_mux_slave_3b <= 4'b001;
							end
						32'b0xxx0xxx0xxx0xxx0xxx0xxx0xxx1000:
							begin
								slave_master_mux_slave<=8'b00000001;
								slave_master_mux_slave_3b <= 4'b000;
							end
						32'b01000xxx0xxx0xxx0xxx0xxx0xxx0xxx:
							begin
								slave_master_mux_slave<=8'b10000000;
								slave_master_mux_slave_3b <= 4'b111;
							end
						32'b00xx01000xxx0xxx0xxx0xxx0xxx0xxx:
							begin
								slave_master_mux_slave<=8'b01000000;
								slave_master_mux_slave_3b <= 4'b110;
							end
						32'b00xx00xx01000xxx0xxx0xxx0xxx0xxx:
							begin
								slave_master_mux_slave<=8'b00100000;
								slave_master_mux_slave_3b <= 4'b101;
							end
						32'b00xx00xx00xx01000xxx0xxx0xxx0xxx:
							begin
								slave_master_mux_slave<=8'b00010000;
								slave_master_mux_slave_3b <= 4'b100;
							end
						32'b00xx00xx00xx00xx01000xxx0xxx0xxx:
							begin
								slave_master_mux_slave<=8'b00001000;
								slave_master_mux_slave_3b <= 4'b011;
							end
						32'b00xx00xx00xx00xx00xx01000xxx0xxx:
							begin
								slave_master_mux_slave<=8'b00000100;
								slave_master_mux_slave_3b <= 4'b010;
							end
						32'b00xx00xx00xx00xx00xx00xx01000xxx:
							begin
								slave_master_mux_slave<=8'b00000010;
								slave_master_mux_slave_3b <= 4'b001;
							end
						32'b00xx00xx00xx00xx00xx00xx00xx0100:
							begin
								slave_master_mux_slave<=8'b00000001;
								slave_master_mux_slave_3b <= 4'b000;
							end
						
						
						32'b001000xx00xx00xx00xx00xx00xx00xx:
							begin
								slave_master_mux_slave<=8'b10000000;
								slave_master_mux_slave_3b <= 4'b111;
							end
						32'b000x001000xx00xx00xx00xx00xx00xx:
							begin
								slave_master_mux_slave<=8'b01000000;
								slave_master_mux_slave_3b <= 4'b110;
							end
						32'b000x000x001000xx00xx00xx00xx00xx:
							begin
								slave_master_mux_slave<=8'b00100000;
								slave_master_mux_slave_3b <= 4'b101;
							end
						32'b000x000x000x001000xx00xx00xx00xx:
							begin
								slave_master_mux_slave<=8'b00010000;
								slave_master_mux_slave_3b <= 4'b100;
							end
						32'b000x000x000x000x001000xx00xx00xx:
							begin
								slave_master_mux_slave<=8'b00001000;
								slave_master_mux_slave_3b <= 4'b011;
							end
						32'b000x000x000x000x000x001000xx00xx:
							begin
								slave_master_mux_slave<=8'b00000100;
								slave_master_mux_slave_3b <= 4'b010;
							end
						32'b000x000x000x000x000x000x001000xx:
							begin
								slave_master_mux_slave<=8'b00000010;
								slave_master_mux_slave_3b <= 4'b001;
							end
						32'b000x000x000x000x000x000x000x0010:
							begin
								slave_master_mux_slave<=8'b00000001;
								slave_master_mux_slave_3b <= 4'b000;
							end
						32'b0001000x000x000x000x000x000x000x:
							begin
								slave_master_mux_slave<=8'b10000000;
								slave_master_mux_slave_3b <= 4'b111;
							end
						32'b00000001000x000x000x000x000x000x:
							begin
								slave_master_mux_slave<=8'b01000000;
								slave_master_mux_slave_3b <= 4'b110;
							end
						32'b000000000001000x000x000x000x000x:
							begin
								slave_master_mux_slave<=8'b00100000;
								slave_master_mux_slave_3b <= 4'b101;
							end
						32'b0000000000000001000x000x000x000x:
							begin
								slave_master_mux_slave<=8'b00010000;
								slave_master_mux_slave_3b <= 4'b100;
							end
						32'b00000000000000000001000x000x000x:
							begin
								slave_master_mux_slave<=8'b00001000;
								slave_master_mux_slave_3b <= 4'b011;
							end
						32'b000000000000000000000001000x000x: //// b00000100  // b010
							begin
								slave_master_mux_slave<=8'b00000100;
								slave_master_mux_slave_3b <= 4'b010;
							end
						32'b0000000000000000000000000001000x:
							begin
								slave_master_mux_slave<=8'b00000010;
								slave_master_mux_slave_3b <= 4'b001;
							end
						32'b00000000000000000000000000000001:
							begin
								slave_master_mux_slave<=8'b00000001;
								slave_master_mux_slave_3b <= 4'b000;
							end
						default
							begin
								slave_master_mux_slave<=8'b0;
								slave_master_mux_slave_3b <= 4'b1000;
							end
					endcase
				end
		end
	
	//*************************** PIPELINE STAGE 4: TRANSFER ACK AND DATA RO RESPECTIV MASTER FROM SLAVE PORTS WITH PARALLEL TO SERIAL CONVERSION *************************//
	
	always @(posedge clk or negedge reset_n) 
		begin
			if(!reset_n)
				begin
					s_p_conversion_counter_4 <= 4'b0;
					s_p_conversion_value_4 <= 4'b0;
					stage_4_state <= 1'b0;
					shift_reg_slave <= {(DATA_WIDTH + 2){1'b0}};//(1'b0)*(DATA_WIDTH + 2);
					last_state_stage_3_4<=8'b0;
					last_state_stage_3_4_single<=8'b0;
					master_enable<=4'b0;
					slave_master_mux_slave_3b_tem<=4'b0;
				end
			else
				begin
					case(stage_4_state)
						1'b0:
							begin
								last_state_stage_3_4_single<=8'b0;
								s_p_conversion_counter_4 <= 4'b0;
								if (readvalid_avs_reg[slave_master_mux_slave_3b])
									begin
										shift_reg_slave <= {readvalid_avs_reg[slave_master_mux_slave_3b], writevalid_avs_reg[slave_master_mux_slave_3b], readdata_avs_reg[slave_master_mux_slave_3b]};
										s_p_conversion_value_4 <= DATA_WIDTH+1'b1;
									end
								else
									begin
										shift_reg_slave <= {readvalid_avs_reg[slave_master_mux_slave_3b], writevalid_avs_reg[slave_master_mux_slave_3b],{DATA_WIDTH{1'b0}}};
										s_p_conversion_value_4 <= 4'd1;
									end
									
								if(slave_master_mux_slave!=8'b0)
									begin
										stage_4_state <= 1'b1;
										last_state_stage_3_4<=(last_state_stage_3_4^slave_releaser_stage)| (1'b1 << slave_master_mux_slave_3b);
										master_enable<=mux_demux_slave_corresponding_master[slave_master_mux_slave_3b];
									end
								else
									begin
										stage_4_state <= stage_4_state;
										last_state_stage_3_4<=(last_state_stage_3_4^slave_releaser_stage);
										master_enable<=4'b0;
									end
								slave_master_mux_slave_3b_tem<=slave_master_mux_slave_3b;
							end
						1'b1:
							begin
								slave_master_mux_slave_3b_tem<=slave_master_mux_slave_3b_tem;
								master_enable<=4'b0;
								shift_reg_slave[DATA_WIDTH + 1:0] <= {shift_reg_slave[DATA_WIDTH:0],1'b0};
								last_state_stage_3_4<=(last_state_stage_3_4^slave_releaser_stage);
								if(s_p_conversion_counter_4 == s_p_conversion_value_4)
									begin
										stage_4_state <= 1'b0;
										s_p_conversion_counter_4 <= 4'b0;
										last_state_stage_3_4_single<=(1'b1 << slave_master_mux_slave_3b_tem);
									end
								else
									begin
										stage_4_state <= stage_4_state;
										s_p_conversion_counter_4 <= s_p_conversion_counter_4 + 4'b1;
										last_state_stage_3_4_single<=8'b0;
									end
							end
					endcase
				end
		end
		
		//*************************** PIPELINE STAGE 5: RELeasE THE MASTER AND SLAVE *************************//
		
		always @(posedge clk or negedge reset_n) // stage 5 (releaser)
			begin
				if (~reset_n)
					begin
						master_releaser_stage<=4'b0;
						slave_releaser_stage<=8'b0;
					end
				else
					begin
						case(last_state_stage_3_4_single)
							8'b10000000:
								begin
									master_releaser_stage<=mux_demux_slave_corresponding_master[7];
									slave_releaser_stage<=8'b10000000;
								end
							8'b01000000:
								begin
									master_releaser_stage<=mux_demux_slave_corresponding_master[6];
									slave_releaser_stage<=8'b01000000;
								end
							8'b00100000:
								begin
									master_releaser_stage<=mux_demux_slave_corresponding_master[5];
									slave_releaser_stage<=8'b00100000;
								end
							8'b00010000:
								begin
									master_releaser_stage<=mux_demux_slave_corresponding_master[4];
									slave_releaser_stage<=8'b00010000;
								end
							8'b00001000:
								begin
									master_releaser_stage<=mux_demux_slave_corresponding_master[3];
									slave_releaser_stage<=8'b00001000;
								end
							8'b00000100:
								begin
									master_releaser_stage<=mux_demux_slave_corresponding_master[2];
									slave_releaser_stage<=8'b00000100;
								end
							8'b00000010:
								begin
									master_releaser_stage<=mux_demux_slave_corresponding_master[1];
									slave_releaser_stage<=8'b00000010;
								end
							8'b00000001:
								begin
									master_releaser_stage<=mux_demux_slave_corresponding_master[0];
									slave_releaser_stage<=8'b00000001;
								end
							default:
								begin
									master_releaser_stage<=4'b0;
									slave_releaser_stage<=8'b0;
								end
						endcase		
					end
			end
	
endmodule
