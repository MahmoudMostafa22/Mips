--------------------------------------------------------------------------------
-- Company: 
-- Engineer:
--
-- Create Date:   11:07:39 04/04/2022
-- Design Name:   
-- Module Name:   D:/svdfsg/ShiftLeft2Bit/test.vhd
-- Project Name:  ShiftLeft2Bit
-- Target Device:  
-- Tool versions:  
-- Description:   
-- 
-- VHDL Test Bench Created by ISE for module: ShiftLeft
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
 
    COMPONENT ShiftLeft
    PORT(
         Inshift : IN  std_logic_vector(31 downto 0);
         Outshift : OUT  std_logic_vector(31 downto 0)
        );
    END COMPONENT;
    

   --Inputs
   signal Inshift : std_logic_vector(31 downto 0) := (others => '0');

 	--Outputs
   signal Outshift : std_logic_vector(31 downto 0);
   -- No clocks detected in port list. Replace <clock> below with 
   -- appropriate port name 
 

 
BEGIN
 
	-- Instantiate the Unit Under Test (UUT)
   uut: ShiftLeft PORT MAP (
          Inshift => Inshift,
          Outshift => Outshift
        );

 

   -- Stimulus process
   stim_proc: process
   begin		
	Inshift <= X"00000001"; wait for 100 ns;
	Inshift <= X"00000002"; wait for 100 ns;
   end process;

END;
