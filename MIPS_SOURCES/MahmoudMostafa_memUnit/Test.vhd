--------------------------------------------------------------------------------
-- Company: 
-- Engineer:
--
-- Create Date:   11:40:23 03/28/2022
-- Design Name:   
-- Module Name:   D:/MahmoudMostafa/MahmoudMostafa_memUnit/Test.vhd
-- Project Name:  MahmoudMostafa_memUnit
-- Target Device:  
-- Tool versions:  
-- Description:   
-- 
-- VHDL Test Bench Created by ISE for module: MemoryUnit
-- 
-- Dependencies:
-- 
-- Revision:
-- Revision 0.01 - File Created
-- Additional Comments:
--
-- Notes: 
-- This testbench has been automatically generated using types std_logic and
-- std_logic_vector for the ports of the unit under test.  Xilinx recommends
-- that these types always be used for the top-level I/O of a design in order
-- to guarantee that the testbench will bind correctly to the post-implementation 
-- simulation model.
--------------------------------------------------------------------------------
LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
 
-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--USE ieee.numeric_std.ALL;
 
ENTITY Test IS
END Test;
 
ARCHITECTURE behavior OF Test IS 
 
    -- Component Declaration for the Unit Under Test (UUT)
 
    COMPONENT MemoryUnit
    PORT(
         MemRead : IN  std_logic;
         MemWrite : IN  std_logic;
         Address : IN  std_logic_vector(31 downto 0);
         DataWrite : IN  std_logic_vector(31 downto 0);
         DataRead : OUT  std_logic_vector(31 downto 0);
         CLK : IN  std_logic
        );
    END COMPONENT;
    

   --Inputs
   signal MemRead : std_logic := '0';
   signal MemWrite : std_logic := '0';
   signal Address : std_logic_vector(31 downto 0) := (others => '0');
   signal DataWrite : std_logic_vector(31 downto 0) := (others => '0');
   signal CLK : std_logic := '0';

 	--Outputs
   signal DataRead : std_logic_vector(31 downto 0);

   -- Clock period definitions
   constant CLK_period : time := 10 ns;
 
BEGIN
 
	-- Instantiate the Unit Under Test (UUT)
   uut: MemoryUnit PORT MAP (
          MemRead => MemRead,
          MemWrite => MemWrite,
          Address => Address,
          DataWrite => DataWrite,
          DataRead => DataRead,
          CLK => CLK
        );

   -- Clock process definitions
   CLK_process :process
   begin
		CLK <= '0';
		wait for CLK_period/2;
		CLK <= '1';
		wait for CLK_period/2;
   end process;
 

   -- Stimulus process
   stim_proc: process
   begin		
      wait for 100 ns;
		wait for CLK_period*10;
		MemRead <= '0' ;MemWrite <= '1';
		Address <= x"00000000";
		DataWrite <= x"00000011";
		wait for CLK_period*10;
		MemRead <= '0' ;MemWrite <= '1';
		Address <= x"00000001";
		DataWrite <= x"00000013";
		wait for CLK_period*10;
		MemRead <= '0' ;MemWrite <= '1';
		Address <= x"00000002";
		DataWrite <= x"00000012";
		wait for CLK_period*10;
		MemRead <= '1';MemWrite <= '0';
		Address <= x"00000000";
		wait for CLK_period*10;
		MemRead <= '1';MemWrite <= '0';
		Address <= x"00000001";
		wait for CLK_period*10;	
		MemRead <= '1';MemWrite <= '0';
		Address <= x"00000002";


   end process;

END;
