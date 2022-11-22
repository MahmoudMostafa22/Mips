--------------------------------------------------------------------------------
-- Company: 
-- Engineer:
--
-- Create Date:   15:48:07 05/06/2022
-- Design Name:   
-- Module Name:   D:/Documents/Year 3/Term 6/Xilinx Programs/Mips/PC/testtest.vhd
-- Project Name:  PC
-- Target Device:  
-- Tool versions:  
-- Description:   
-- 
-- VHDL Test Bench Created by ISE for module: PCCounter
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
USE IEEE.STD_LOGIC_ARITH.ALL;
USE IEEE.STD_LOGIC_UNSIGNED.ALL;
 
-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--USE ieee.numeric_std.ALL;
 
ENTITY testtest IS
END testtest;
 
ARCHITECTURE behavior OF testtest IS 
 
    -- Component Declaration for the Unit Under Test (UUT)
 
    COMPONENT PCCounter
    PORT(
         A: IN  std_logic_vector(31 downto 0);
         Y : OUT  std_logic_vector(31 downto 0);
			CLK : in  STD_LOGIC
        );
    END COMPONENT;
    

   --Inputs
   signal A : std_logic_vector(31 downto 0) := (others => '0');
	signal CLK : std_logic := '0';
 	--Outputs
   signal Y : std_logic_vector(31 downto 0);
   -- No clocks detected in port list. Replace <clock> below with 
   -- appropriate port name 
 
   constant CLK_period : time := 10 ns;
 
BEGIN
 
	-- Instantiate the Unit Under Test (UUT)
   uut: PCCounter PORT MAP (
          A => A,
          Y => Y,
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
      -- hold reset state for 100 ns.
      A <= X"00000000"; wait for CLK_period*10;
		A <= X"00000001"; wait for CLK_period*10;
		A <= X"00000002"; wait for CLK_period*10;
		A <= X"00000003"; wait for CLK_period*10;
		A <= X"00000004"; wait for CLK_period*10;
		A <= X"00000005"; wait for CLK_period*10;

      -- insert stimulus here 

      wait;
   end process;

END;
