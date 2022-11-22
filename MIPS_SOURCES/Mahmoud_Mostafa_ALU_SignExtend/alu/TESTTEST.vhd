--------------------------------------------------------------------------------
-- Company: 
-- Engineer:
--
-- Create Date:   11:23:35 02/28/2022
-- Design Name:   
-- Module Name:   D:/mahmo/alu/TESTTEST.vhd
-- Project Name:  alu
-- Target Device:  
-- Tool versions:  
-- Description:   
-- 
-- VHDL Test Bench Created by ISE for module: ALU4Bit
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
 
ENTITY TESTTEST IS
END TESTTEST;
 
ARCHITECTURE behavior OF TESTTEST IS 
 
    -- Component Declaration for the Unit Under Test (UUT)
 
    COMPONENT ALU4Bit
    PORT(
         A : IN  std_logic_vector(31 downto 0);
         B : IN  std_logic_vector(31 downto 0);
         ALUin : IN  std_logic_vector(3 downto 0);
         zero : OUT  std_logic;
         result : OUT  std_logic_vector(31 downto 0)
        );
    END COMPONENT;
    

   --Inputs
   signal A : std_logic_vector(31 downto 0) := (others => '0');
   signal B : std_logic_vector(31 downto 0) := (others => '0');
   signal ALUin : std_logic_vector(3 downto 0) := (others => '0');

 	--Outputs
   signal zero : std_logic;
   signal result : std_logic_vector(31 downto 0);
   -- No clocks detected in port list. Replace <clock> below with 
   -- appropriate port name 
 

 
BEGIN
 
	-- Instantiate the Unit Under Test (UUT)
   uut: ALU4Bit PORT MAP (
          A => A,
          B => B,
          ALUin => ALUin,
          zero => zero,
          result => result
        );

   -- Stimulus process
   stim_proc: process
   begin		
      wait for 100 ns;	
		A <= X"0000000A";
		B <= X"00000001";
		ALUin <= "0000";
		wait for 100 ns;
		ALUin <= "0001";
		wait for 100 ns;
		ALUin <= "0111";
		wait for 100 ns;
		ALUin <= "0010";
		wait for 100 ns;
		ALUin <= "0110";
		wait for 100 ns;
		A <= X"00000001";
		B <= X"00000001";
		ALUin <= "1100";
		wait for 100 ns;


   end process;

END;
