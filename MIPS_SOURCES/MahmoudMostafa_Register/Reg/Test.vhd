--------------------------------------------------------------------------------
-- Company: 
-- Engineer:
--
-- Create Date:   11:27:07 03/21/2022
-- Design Name:   
-- Module Name:   D:/MahmoudMostafa_Register/Reg/Test.vhd
-- Project Name:  Reg
-- Target Device:  
-- Tool versions:  
-- Description:   
-- 
-- VHDL Test Bench Created by ISE for module: RegisterFile
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
 
    COMPONENT RegisterFile
    PORT(
         ReadReg1 : IN  std_logic_vector(4 downto 0);
         ReadReg2 : IN  std_logic_vector(4 downto 0);
         WriteReg : IN  std_logic_vector(4 downto 0);
         WriteData : IN  std_logic_vector(31 downto 0);
         RegWrite : IN  std_logic;
         ReadData1 : OUT  std_logic_vector(31 downto 0);
         ReadData2 : OUT  std_logic_vector(31 downto 0);
			CLK : in  STD_LOGIC);
    END COMPONENT;
    

   --Inputs
   signal ReadReg1 : std_logic_vector(4 downto 0) := (others => '0');
   signal ReadReg2 : std_logic_vector(4 downto 0) := (others => '0');
   signal WriteReg : std_logic_vector(4 downto 0) := (others => '0');
   signal WriteData : std_logic_vector(31 downto 0) := (others => '0');
   signal RegWrite : std_logic := '0';
	signal CLK : std_logic := '0';

 	--Outputs
   signal ReadData1 : std_logic_vector(31 downto 0);
   signal ReadData2 : std_logic_vector(31 downto 0);
   -- No clocks detected in port list. Replace <clock> below with 
   -- appropriate port name 
 constant CLK_period : time := 10 ns;

 
BEGIN
 
	-- Instantiate the Unit Under Test (UUT)
   uut: RegisterFile PORT MAP (
          ReadReg1 => ReadReg1,
          ReadReg2 => ReadReg2,
          WriteReg => WriteReg,
          WriteData => WriteData,
          RegWrite => RegWrite,
          ReadData1 => ReadData1,
          ReadData2 => ReadData2,
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
   wait for clk_period*10;
	ReadReg1 <= "00000";
	ReadReg2 <= "00001";
	wait for 100 ns;
	RegWrite <= '1';
	WriteReg <= "00001";
	WriteData <= X"00000002";
	wait for 100 ns;
	ReadReg1 <= "00001";
	
   end process;

END;
