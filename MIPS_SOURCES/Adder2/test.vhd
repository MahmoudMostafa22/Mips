--------------------------------------------------------------------------------
-- Company: 
-- Engineer:
--
-- Create Date:   16:12:10 05/06/2022
-- Design Name:   
-- Module Name:   D:/Documents/Year 3/Term 6/Xilinx Programs/Mips/Adder2/test.vhd
-- Project Name:  Adder2
-- Target Device:  
-- Tool versions:  
-- Description:   
-- 
-- VHDL Test Bench Created by ISE for module: Adder4Bit
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
 
ENTITY test IS
END test;
 
ARCHITECTURE behavior OF test IS 
 
    -- Component Declaration for the Unit Under Test (UUT)
 
    COMPONENT Adder4Bit
    PORT(
         INPUT1 : IN  std_logic_vector(31 downto 0);
         OUTPUT1 : OUT  std_logic_vector(31 downto 0)
        );
    END COMPONENT;
    

   --Inputs
   signal INPUT1 : std_logic_vector(31 downto 0) := (others => '0');

 	--Outputs
   signal OUTPUT1 : std_logic_vector(31 downto 0);
   -- No clocks detected in port list. Replace <clock> below with 
   -- appropriate port name 
 
   
 
BEGIN
 
	-- Instantiate the Unit Under Test (UUT)
   uut: Adder4Bit PORT MAP (
          INPUT1 => INPUT1,
          OUTPUT1 => OUTPUT1
        );

 

   -- Stimulus process
   stim_proc: process
   begin		
      -- hold reset state for 100 ns.
		INPUT1 <= X"00000000";
      wait for 100 ns;	
		INPUT1 <= X"00000001";
      wait for 100 ns;	
		INPUT1 <= X"00000002";
      wait for 100 ns;	
   end process;

END;
