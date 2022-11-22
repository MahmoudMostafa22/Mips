--------------------------------------------------------------------------------
-- Company: 
-- Engineer:
--
-- Create Date:   08:32:17 03/25/2022
-- Design Name:   
-- Module Name:   D:/Documents/Year 3/Term 6/Xilinx Programs/Week 2/Alu_control/Test.vhd
-- Project Name:  Alu_control
-- Target Device:  
-- Tool versions:  
-- Description:   
-- 
-- VHDL Test Bench Created by ISE for module: ALU_Control
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
 
    COMPONENT ALU_Control
    PORT(
         ALUOP : IN  std_logic_vector(1 downto 0);
         FuncField : IN  std_logic_vector(5 downto 0);
         Operation : OUT  std_logic_vector(3 downto 0)
        );
    END COMPONENT;
    

   --Inputs
   signal ALUOP : std_logic_vector(1 downto 0) := (others => '0');
   signal FuncField : std_logic_vector(5 downto 0) := (others => '0');

 	--Outputs
   signal Operation : std_logic_vector(3 downto 0);
   -- No clocks detected in port list. Replace <clock> below with 
   -- appropriate port name 
 
BEGIN
 
	-- Instantiate the Unit Under Test (UUT)
   uut: ALU_Control PORT MAP (
          ALUOP => ALUOP,
          FuncField => FuncField,
          Operation => Operation
        );

   -- Stimulus process
   stim_proc: process
   begin		
      ALUOP <= "00"; wait for 100 ns;
		ALUOP <= "01"; wait for 100 ns;
		ALUOP <= "10"; FuncField <="010000"; wait for 100 ns;
		ALUOP <= "10"; FuncField <="110100"; wait for 100 ns;
		ALUOP <= "10"; FuncField <="000101"; wait for 100 ns;
		ALUOP(1)<= '1'; FuncField <="110010"; wait for 100 ns;
		ALUOP(1)<= '1'; FuncField <="101010"; wait for 100 ns;
   
	end process;

END;
