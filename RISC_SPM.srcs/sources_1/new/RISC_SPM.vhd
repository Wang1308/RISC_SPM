----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date: 28.10.2023 22:57:32
-- Design Name: 
-- Module Name: RISC_SPM - Behavioral
-- Project Name: 
-- Target Devices: 
-- Tool Versions: 
-- Description: 
-- 
-- Dependencies: 
-- 
-- Revision:
-- Revision 0.01 - File Created
-- Additional Comments:
-- 
----------------------------------------------------------------------------------


library ieee;
use ieee.std_logic_1164.all;
use IEEE.STD_LOGIC_UNSIGNED.ALL;
use ieee.NUMERIC_STD.all;
entity RISC_SPM is

 port ( clk_P, rst_P : in std_logic);
 end RISC_SPM;
 
 architecture arch of RISC_SPM is
   
   component processing_unit
	  port (mem_word  : in std_logic_vector ( 7 downto 0);
       load_R0, load_R1, load_R2, load_R3, load_PC, Inc_PC : in std_logic;
		 sel_bus_1_mux : in std_logic_vector (2 downto 0);
		 sel_bus_2_mux : in std_logic_vector (1 downto 0);
		 load_IR, load_Add_R, load_Reg_Y, load_Reg_Z : in std_logic;
		 clk, rst : in std_logic;
		 instruction, address, Bus_1 : out std_logic_vector (7 downto 0);
		 Zflag : out std_logic );
		end component;
	component controller_unit 
     port (instruction : in std_logic_vector (7 downto 0);
        zero : in std_logic;
		  clk, rst : in std_logic;
		  load_R0, load_R1, load_R2, load_R3, load_PC, Inc_PC : out std_logic;  
		  sel_bus_1_mux : out std_logic_vector (2 downto 0);
		 sel_bus_2_mux : out std_logic_vector (1 downto 0);
		 load_IR, load_Add_R, load_Reg_Y, load_Reg_Z, write_Mem : out std_logic);	
		end component;
	component memory_unit
     port (data_in : in std_logic_vector (7 downto 0);
		 data_out : out std_logic_vector (7 downto 0);
		 address : in std_logic_vector (7 downto 0);
		 clk, write_Mem : in std_logic);
	end component;
   signal    mem_word_P  : std_logic_vector ( 7 downto 0);
   signal    load_R0_P, load_R1_P, load_R2_P, load_R3_P, load_PC_P, Inc_PC_P : std_logic;
	signal	 sel_bus_1_mux_P :  std_logic_vector (2 downto 0);
	signal	 sel_bus_2_mux_P: std_logic_vector (1 downto 0);
	signal	 load_IR_P, load_Add_R_P, load_Reg_Y_P, load_Reg_Z_P :  std_logic;
	signal	 instruction_P, address_P, Bus_1_P : std_logic_vector (7 downto 0);
	signal	 Zflag_P :  std_logic; 
   signal    write_Mem_P: std_logic;
  
	begin
	
   
     unit1: processing_unit
	      port map (mem_word => mem_word_P, load_R0 => load_R0_P, load_R1 => load_R1_P, load_R2 => load_R2_P, load_R3 => load_R3_P, load_PC => load_PC_P, Inc_PC => Inc_PC_P,
			          load_IR => load_IR_P, load_Add_R => load_Add_R_P, load_Reg_Y => load_Reg_Y_P, load_Reg_Z => load_Reg_Z_P, sel_bus_1_mux => sel_bus_1_mux_P, sel_bus_2_mux => sel_bus_2_mux_P,
						 clk =>clk_P, rst => rst_P, instruction => instruction_P, address => address_P, Bus_1 => Bus_1_P, Zflag => Zflag_P ); 
	  unit2 : controller_unit
         port map (instruction => instruction_P, zero => Zflag_P, clk => clk_P , rst => rst_P,load_R0 => load_R0_P, load_R1 => load_R1_P, load_R2 => load_R2_P, load_R3 => load_R3_P, load_PC => load_PC_P, Inc_PC => Inc_PC_P,
			          load_IR => load_IR_P, load_Add_R => load_Add_R_P, load_Reg_Y => load_Reg_Y_P, load_Reg_Z => load_Reg_Z_P, sel_bus_1_mux => sel_bus_1_mux_P, sel_bus_2_mux => sel_bus_2_mux_P, write_Mem => write_Mem_P);
		unit3 : memory_unit
         port map ( data_in => Bus_1_P, data_out => mem_word_P, address => address_P, clk => clk_P, write_Mem => write_Mem_P);
	end arch;
--Code Processing unit 
library ieee;
use ieee.std_logic_1164.all;
use IEEE.STD_LOGIC_UNSIGNED.ALL;
use ieee.NUMERIC_STD.all;

entity processing_unit is
port ( mem_word  : in std_logic_vector ( 7 downto 0);
       load_R0, load_R1, load_R2, load_R3, load_PC, Inc_PC : in std_logic;
		 sel_bus_1_mux : in std_logic_vector (2 downto 0);
		 sel_bus_2_mux : in std_logic_vector (1 downto 0);
		 load_IR, load_Add_R, load_Reg_Y, load_Reg_Z : in std_logic;
		 clk, rst : in std_logic;
		 instruction, address, Bus_1 : out std_logic_vector (7 downto 0);
		 Zflag : out std_logic );
end processing_unit;

architecture str_arch of processing_unit is
   component Register_unit
    port ( data_in : in std_logic_vector ( 7 downto 0);
	         load, clk, rst : in std_logic;
				data_out : out std_logic_vector (7 downto 0));
		end component;

    component D_flop
     port ( data_in : in std_logic;
	         load, clk, rst : in std_logic;
				data_out : out std_logic);	 
		end component;

     component Address_Register
	   port ( data_in : in std_logic_vector ( 7 downto 0);
	         load, clk, rst : in std_logic;
				data_out : out std_logic_vector (7 downto 0));
		end component;
	
	  component Instruction_Register
	   port ( data_in : in std_logic_vector ( 7 downto 0);
	         load, clk, rst : in std_logic;
				data_out : out std_logic_vector (7 downto 0));
		end component;
		
     component Program_Counter
	   port ( data_in : in std_logic_vector ( 7 downto 0);
	         load_PC, Inc_PC, clk, rst : in std_logic;
				count : out std_logic_vector ( 7 downto 0)); 
		end component;

     component Mux_5_1
	    port (data_a, data_b, data_c, data_d, data_e : in  std_logic_vector ( 7 downto 0);
		                               sel    : in  std_logic_vector ( 2 downto 0);
												 mux_out: out std_logic_vector ( 7 downto 0));  
		end component;

		component Mux_3_1
	    port (data_a, data_b, data_c : in  std_logic_vector ( 7 downto 0);
		                       sel    : in  std_logic_vector ( 1 downto 0);
									  mux_out: out std_logic_vector ( 7 downto 0));  
		end component;
		
		component Alu_RISC
		 port (data_1, data_2 : in std_logic_vector( 7 downto 0);
		                  sel : in std_logic_vector( 3 downto 0);
							alu_out: out std_logic_vector( 7 downto 0);
							alu_zero_flag: out std_logic);
		end component;				
		
		signal Bus_2, R0_out, R1_out, R2_out, R3_out: std_logic_vector( 7 downto 0);
		signal Y_value, PC_count, alu_out, Bus1, instruction1: std_logic_vector( 7 downto 0);
		signal alu_zero_flag: std_logic;
		signal opcode: std_logic_vector( 3 downto 0);
		begin	
		   R0: Register_unit
		      port map ( data_in => Bus_2, load => load_R0, clk =>clk, rst => rst, data_out => R0_out);
			R1: Register_unit
		      port map ( data_in => Bus_2, load => load_R1, clk =>clk, rst => rst, data_out => R1_out);
			R2: Register_unit
		      port map ( data_in => Bus_2, load => load_R2, clk =>clk, rst => rst, data_out => R2_out);	
			R3: Register_unit
		      port map ( data_in => Bus_2, load => load_R3, clk =>clk, rst => rst, data_out => R3_out);	
			Reg_Y: Register_unit
		      port map ( data_in => Bus_2, load => load_Reg_Y, clk =>clk, rst => rst, data_out => Y_value);
			Red_Z: D_flop
		      port map ( data_in => alu_zero_flag, load => load_Reg_Z, clk =>clk, rst => rst, data_out => Zflag);	
			Add_R: Address_Register
		      port map ( data_in => Bus_2, load => load_Add_R, clk =>clk, rst => rst, data_out => address);	
			IR: Instruction_Register
		      port map ( data_in => Bus_2, load => load_IR, clk =>clk, rst => rst, data_out => instruction1);
		   PC: Program_Counter
			   port map ( data_in => Bus_2, load_PC => load_PC, Inc_PC => Inc_PC, clk =>clk, rst => rst, count => PC_count);
			MUX_1: mux_5_1
		      port map ( data_a => R0_out, data_b => R1_out, data_c => R2_out, data_d => R3_out, data_e => PC_count, sel => sel_bus_1_mux, mux_out => Bus1);
			MUX_2: mux_3_1
		      port map ( data_a => alu_out, data_b => Bus1, data_c => mem_word, sel => sel_bus_2_mux, mux_out => Bus_2);
			ALU: Alu_RISC
		      port map ( data_1 => Y_value, data_2 => Bus1, sel => opcode, alu_out => alu_out, alu_zero_flag => alu_zero_flag);
				instruction <= instruction1;
				opcode <= instruction1(7 downto 4);
				Bus_1 <= Bus1;
		end str_arch;
		
		
	-- Code cua Register_unit	
	library ieee;
use ieee.std_logic_1164.all;
	entity Register_unit is 
   port ( data_in : in std_logic_vector (7 downto 0);
	         load, clk, rst : in std_logic;
				data_out : out std_logic_vector (7 downto 0));
	end Register_unit;
   architecture arch of Register_unit is
     begin 
	    process(clk, rst)
		  begin 
		    if rst = '0'    then data_out <= (others => '0');
			elsif clk'event and clk = '1' and load = '1' then data_out <= data_in;
			end if; 
		end process;
	end arch;
	
	-- Code cua D_flop
	library ieee;
use ieee.std_logic_1164.all;
   entity D_flop is
   port ( data_in : in std_logic;
	         load, clk, rst : in std_logic;
				data_out : out std_logic);
	end D_flop;
   architecture arch of D_flop is
    	begin 
	    process(clk, rst)
		  begin 
		    if rst = '0'    then data_out <= '0';
			elsif clk'event and clk = '1' and load = '1' then data_out <= data_in;
			end if; 
		end process;
	end arch;
	
	-- Code cua Address_Register
	library ieee;
use ieee.std_logic_1164.all;
	entity Address_Register is 
	port ( data_in : in std_logic_vector ( 7 downto 0);
	         load, clk, rst : in std_logic;
				data_out : out std_logic_vector (7 downto 0));
	end Address_Register;
	architecture arch of Address_Register is
    begin 
	    process(clk, rst)
		  begin 
		    if rst = '0'    then data_out <= (others => '0');
			elsif clk'event and clk = '1' and load = '1' then data_out <= data_in;
			end if; 
		end process;
	end arch;

   -- Code cua Instruction_Register
	library ieee;
use ieee.std_logic_1164.all;
   entity Instruction_Register is
	port ( data_in : in std_logic_vector ( 7 downto 0);
	         load, clk, rst : in std_logic;
				data_out : out std_logic_vector (7 downto 0));
		end Instruction_Register;
	architecture arch of Instruction_Register is
	begin 
	    process(clk, rst)
		  begin 
		    if rst = '0'    then data_out <= (others => '0');
			elsif clk'event and clk = '1' and load = '1' then data_out <= data_in;
			end if; 
		end process;
	 end arch;
	 
	 -- Code cua Program_Counter
	 library ieee;
use ieee.std_logic_1164.all;
use ieee.NUMERIC_STD.all;
	 entity Program_Counter is
	 port ( data_in : in std_logic_vector ( 7 downto 0);
	         load_PC, Inc_PC, clk, rst : in std_logic;
				count : out std_logic_vector ( 7 downto 0)); 
		end Program_Counter;
	 architecture arch of Program_Counter is
	 signal PC_reg : std_logic_vector(7 downto 0);
	 begin
	  process(clk, rst)
		begin 
		    if rst = '0'    then PC_reg <= (others => '0');
			elsif (clk'event) and (clk = '1') then    if load_PC = '1' then PC_reg <= data_in;
			                                   elsif Inc_PC  = '1' then PC_reg <= std_logic_vector( signed(PC_reg) + 1) ;
														  end if;
			end if; 
		end process;
		count <= PC_reg;
	 end arch;
	 
	 -- Code Mux_5_1
	 library ieee;
use ieee.std_logic_1164.all;
	 entity Mux_5_1 is
	 port (data_a, data_b, data_c, data_d, data_e : in  std_logic_vector ( 7 downto 0);
		                               sel    : in  std_logic_vector ( 2 downto 0);
												 mux_out: out std_logic_vector ( 7 downto 0));
	 end Mux_5_1;
    architecture arch of Mux_5_1 is
      begin 
       mux_out <= data_a when sel = "000" else 
	               data_b when sel = "001" else
				      data_c when sel = "010" else
						data_d when sel = "011" else 
						data_e when sel = "100" else
						(others => 'X');
		end arch;
     
	  -- Code cua Mux_3_1
	  library ieee;
use ieee.std_logic_1164.all;
	  entity Mux_3_1 is
	  port (data_a, data_b, data_c : in  std_logic_vector ( 7 downto 0);
		                       sel    : in  std_logic_vector ( 1 downto 0);
									  mux_out: out std_logic_vector ( 7 downto 0));  
		end Mux_3_1;
		architecture arch of Mux_3_1 is
		 begin 
       mux_out <= data_a when sel = "00" else 
	               data_b when sel = "01" else
				      data_c when sel = "10" else
						(others => 'X');
		end arch;
	 
    -- Code cua ALU_RISC
	 library ieee;
use ieee.std_logic_1164.all;
use IEEE.STD_LOGIC_UNSIGNED.ALL;
use ieee.NUMERIC_STD.all;
    entity ALU_RISC is
    	 port (data_1, data_2 : in std_logic_vector( 7 downto 0);
		                  sel : in std_logic_vector( 3 downto 0);
							alu_out: out std_logic_vector( 7 downto 0);
							alu_zero_flag: out std_logic);
		end ALU_RISC;
	  architecture arch of ALU_RISC is
	    signal alu_results : std_logic_vector(7 downto 0); 
	  begin 
	    process(sel, data_1, data_2)
		   begin 
		  case sel is
		    when "0000" => alu_results <= (others => '0');
			 when "0001" => alu_results <= data_1 + data_2;
			 when "0010" => alu_results <= data_2 - data_1;
			 when "0011" => alu_results <= data_1 and data_2;
			 when "0100" => alu_results <= not data_2;
			 when others => alu_results <= (others => '0');
			end case;
			if alu_results = "00000000" then alu_zero_flag <= '1';
			else alu_zero_flag <= '0';
			end if;
		end process;
		alu_out <= alu_results;
	end arch;
--Code Controller unit 

library ieee;
use ieee.std_logic_1164.all;

entity controller_unit is
  port (instruction : in std_logic_vector (7 downto 0);
        zero : in std_logic;
		  clk, rst : in std_logic;
		  load_R0, load_R1, load_R2, load_R3, load_PC, Inc_PC : out std_logic;  
		  sel_bus_1_mux : out std_logic_vector (2 downto 0);
		 sel_bus_2_mux : out std_logic_vector (1 downto 0);
		 load_IR, load_Add_R, load_Reg_Y, load_Reg_Z, write_Mem : out std_logic
			);
	end controller_unit;

architecture two_seg_arch of controller_unit is
 type mc_state_type is (S_idle, S_fet1, S_fet2, S_dec, S_ex1, S_rd1, S_rd2, S_wr1, S_wr2, S_br1, S_br2, S_halt);
 signal state_reg, state_next: mc_state_type;
 signal opcode : std_logic_vector (3 downto 0);
 signal sel_R0, sel_R1, sel_R2, sel_R3, sel_PC: std_logic;
 signal sel_ALU, sel_Bus_1, sel_Mem: std_logic;
 signal src, dest : std_logic_vector (1 downto 0);
 signal err_flag: std_logic;
 begin 
   sel_bus_1_mux <= "000" when sel_R0='1' else
	                 "001" when sel_R1='1' else
						  "010" when sel_R2='1' else
						  "011" when sel_R3='1' else
						  "100" when sel_PC='1' else (others => 'X');
	sel_bus_2_mux <= "00" when sel_ALU='1' else
	                 "01" when sel_Bus_1='1' else
						  "10" when sel_Mem='1' else (others => 'X');
						  
   process (clk,rst)
    begin 
	   if rst='0'   then state_reg <= S_idle;
	   elsif clk'event and clk ='1' then state_reg <= state_next;
	   end if;
	end process;
   
	opcode <= instruction(7 downto 4);
	src <= instruction (3 downto 2);
	dest <= instruction (1 downto 0);
   process (state_reg, opcode, src, dest, zero)
     begin 
	   sel_R0 <='0'; sel_R1 <='0'; sel_R2 <='0'; sel_R3 <='0'; sel_PC <='0';
		sel_ALU <='0'; sel_Bus_1 <='0'; sel_Mem <='0';
	   load_R0 <='0';  load_R1 <='0'; load_R2 <='0'; load_R3 <='0'; load_PC <='0'; Inc_PC <='0';
		load_Add_R <='0'; load_IR <='0'; load_Reg_Y <='0'; load_Reg_Z <='0'; write_Mem <='0';
		err_flag <='0';
		
		case state_reg is
		    when S_idle => state_next <= S_fet1;
			 when S_fet1 => state_next <= S_fet2;
			                sel_PC <='1'; sel_Bus_1 <='1'; load_Add_R <='1';
			 when S_fet2 => state_next <= S_dec;
                         sel_Mem <='1'; load_IR <='1';Inc_PC <='1';
			 when S_dec =>  
                             if opcode = "0000" then state_next <= S_fet1;
							        elsif (opcode = "0001") or (opcode = "0010") or (opcode = "0011") then state_next <= S_ex1;
								                                                                            sel_Bus_1 <='1';
																															       load_Reg_Y <='1';
																															       case src is 
																															            when "00" => sel_R0 <='1';
																																         when "01" => sel_R1 <='1';
																																         when "10" => sel_R2 <='1';
																																	      when "11" => sel_R3 <='1';
																																	      when others => err_flag <='1';
																																	end case;
										elsif opcode = "0100" then state_next <= S_fet1;
                                                         load_Reg_Z <='1';
									                              sel_ALU <='1';	
										                           case src is 
																				 when "00" => sel_R0 <='1';
																				 when "01" => sel_R1 <='1';
																				 when "10" => sel_R2 <='1';
																				 when "11" => sel_R3 <='1';
																				 when others => err_flag <='1';
																			end case;
										                           case dest is 
																				 when "00" => load_R0 <='1';
																				 when "01" => load_R1 <='1';
																				 when "10" => load_R2 <='1';
																				 when "11" => load_R3 <='1';
																				 when others => err_flag <='1';
																			end case;	
										elsif opcode = "0101" then state_next <= S_rd1;
																			sel_PC <='1';
																			sel_Bus_1 <='1';
																			load_Add_R <='1';
										elsif opcode = "0110" then	state_next <= S_wr1;
																			sel_PC <='1';
																			sel_Bus_1 <='1';
																			load_Add_R <='1';
										elsif opcode = "0111" then state_next <= S_br1;
																			sel_PC <='1';
																			sel_Bus_1 <='1';
																			load_Add_R <='1';
										elsif opcode = "1000" then if zero ='1' then
																			state_next <= S_br1;
																			sel_PC <='1';
																			sel_Bus_1 <='1';
																			load_Add_R <='1';
																			else state_next <= S_fet1; Inc_PC <='1';
																			end if;
										else state_next <= S_halt;
										end if;
			when S_ex1 => state_next <= S_fet1;
								load_Reg_Z <='1';
								sel_ALU <='1';
								case dest is
								  when "00" => sel_R0 <='1'; load_R0 <='1';
								  when "01" => sel_R1 <='1'; load_R1 <='1';
								  when "10" => sel_R2 <='1'; load_R2 <='1';
								  when "11" => sel_R3 <='1'; load_R3 <='1';
								  when others => err_flag <='1';
								end case;
			when S_rd1 => state_next <= S_rd2;
                       sel_Mem <='1';
							  load_Add_R <='1';
							  Inc_PC <='1';
			when S_wr1 => state_next <= S_wr2;
                       sel_Mem <='1';
							  load_Add_R <='1';
							  Inc_PC <='1';
			when S_rd2 => state_next <= S_fet1;
							  sel_Mem <='1';
							  case dest is 
												when "00" => load_R0 <='1';
												when "01" => load_R1 <='1';
												when "10" => load_R2 <='1';
												when "11" => load_R3 <='1';
												when others => err_flag <='1';
							  end case;
			when S_wr2 => state_next <= S_fet1;
							  write_Mem <='1';
							  case src is 
												when "00" => sel_R0 <='1';
												when "01" => sel_R1 <='1';
												when "10" => sel_R2 <='1';
												when "11" => sel_R3 <='1';
												when others => err_flag <='1';
							  end case;
			when S_br1 => state_next <= S_br2;
							  sel_Mem <='1';
							  load_Add_R <= '1';
			when S_br2 => state_next <= S_fet1;
							  sel_Mem <='1';
							  load_PC <= '1';
			when S_halt => state_next <= S_halt;
			when others => state_next <= S_idle;
			end case;
		end process;
	end two_seg_arch;	
		
--Code Memory unit 

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity memory_unit is
 port (data_in : in std_logic_vector (7 downto 0);
		 data_out : out std_logic_vector (7 downto 0);
		 address : in std_logic_vector (7 downto 0);
		 clk, write_Mem : in std_logic);
	end memory_unit;
architecture arch of memory_unit is
type memory_array is array (255 downto 0) of std_logic_vector (7 downto 0);
signal mem: memory_array:=
  (x"00",x"00",x"00",x"00",x"00",x"00",x"00",x"00",
   x"00",x"00",x"00",x"00",x"00",x"00",x"00",x"00",
   x"00",x"00",x"00",x"00",x"00",x"00",x"00",x"00",
   x"00",x"00",x"00",x"00",x"00",x"00",x"00",x"00",
   x"00",x"00",x"00",x"00",x"00",x"00",x"00",x"00",
   x"00",x"00",x"00",x"00",x"00",x"00",x"00",x"00",
   x"00",x"00",x"00",x"00",x"00",x"00",x"00",x"00",
   x"00",x"00",x"00",x"00",x"00",x"00",x"00",x"00",
   x"00",x"00",x"00",x"00",x"00",x"00",x"00",x"00",
   x"00",x"00",x"00",x"00",x"00",x"00",x"00",x"00",
   x"00",x"00",x"00",x"00",x"00",x"00",x"00",x"00",
   x"00",x"00",x"00",x"00",x"00",x"00",x"00",x"00",
   x"00",x"00",x"00",x"00",x"00",x"00",x"00",x"00",
   x"00",x"00",x"00",x"00",x"00",x"00",x"00",x"00",
   x"00",x"00",x"00",x"09",x"f0",x"00",x"00",x"00",
   x"00",x"8b",x"00",x"00",x"00",x"02",x"01",x"06",
   x"00",x"00",x"00",x"00",x"00",x"00",x"00",x"00",
   x"00",x"00",x"00",x"00",x"00",x"00",x"00",x"00",
   x"00",x"00",x"00",x"00",x"00",x"00",x"00",x"00",
   x"00",x"00",x"00",x"00",x"00",x"00",x"00",x"00",
   x"00",x"00",x"00",x"00",x"00",x"00",x"00",x"00",
   x"00",x"00",x"00",x"00",x"00",x"00",x"00",x"00",
   x"00",x"00",x"00",x"00",x"00",x"00",x"00",x"00",
   x"00",x"00",x"00",x"00",x"00",x"00",x"00",x"00",
   x"00",x"00",x"00",x"00",x"00",x"00",x"00",x"00",
   x"00",x"00",x"00",x"00",x"00",x"00",x"00",x"00",
   x"00",x"00",x"00",x"00",x"00",x"00",x"00",x"00",
   x"00",x"00",x"00",x"00",x"00",x"00",x"00",x"00",
   x"00",x"00",x"00",x"00",x"00",x"00",x"00",x"00",
   x"00",x"00",x"00",x"00",x"00",x"00",x"00",x"00",
   x"00",x"8c",x"73",x"1b",x"86",x"80",x"21",x"81",
   x"50",x"80",x"51",x"83",x"53",x"82",x"52",x"00")
;
 begin 
 data_out <= mem(to_integer (unsigned(address)));
 process (clk)
   begin 
	  if clk'event and clk ='1' and write_Mem ='1' then mem(to_integer(unsigned(address))) <= data_in;
	  end if;
	 end process;
end arch;	 			
