----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date: 28.10.2023 23:10:05
-- Design Name: 
-- Module Name: RISC_SPM_tb - Behavioral
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


library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx leaf cells in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity RISC_SPM_tb is
--  Port ( );
end RISC_SPM_tb;

architecture Behavioral of RISC_SPM_tb is
SIGNAL clk_P : STD_LOGIC := '0';
SIGNAL rst_P : STD_LOGIC;

COMPONENT RISC_SPM
	PORT (
	clk_P : IN STD_LOGIC;
	rst_P : IN STD_LOGIC
	);
END COMPONENT;
constant T: time := 10ns;

begin 
  il: RISC_SPM
  port map ( clk_P => clk_P, rst_P => rst_P);
RAM_CLOCK_process :process
   begin
  clk_P <= '0';
  wait for T;
  clk_P <= '1';
  wait for T;
   end process; 
init : PROCESS                                               
                                     
BEGIN 
        rst_P <= '0';
        wait for T;
        rst_P <= '1';
        wait for T;
WAIT;                                                       
END process;

end Behavioral;
