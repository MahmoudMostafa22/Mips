----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date:    10:55:05 03/21/2022 
-- Design Name: 
-- Module Name:    Register - Behavioral 
-- Project Name: 
-- Target Devices: 
-- Tool versions: 
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
use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx primitives in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity RegisterFile is
    Port ( ReadReg1 : in  STD_LOGIC_VECTOR (4 downto 0);
           ReadReg2 : in  STD_LOGIC_VECTOR (4 downto 0);
           WriteReg : in  STD_LOGIC_VECTOR (4 downto 0);
           WriteData : in  STD_LOGIC_VECTOR (31 downto 0);
           RegWrite : in  STD_LOGIC;
           ReadData1 : out  STD_LOGIC_VECTOR (31 downto 0);
           ReadData2 : out  STD_LOGIC_VECTOR (31 downto 0);
			  CLK : in  STD_LOGIC);
			  
end RegisterFile;

architecture Behavioral of RegisterFile is
type RegtypeFile is array (0 to 31) of STD_LOGIC_VECTOR (31 downto 0);
signal RegArray : RegtypeFile:= (X"00000000",X"00000000",
											X"00000000",X"00000000",
											X"00000005",X"00000007",
											X"00000000",X"00000000",
											X"00000000",X"00000000",
											X"00000000",X"00000000",
											X"00000000",X"00000000",
											X"00000000",X"00000000",
											X"00000000",X"00000000",
											X"00000000",X"00000000",
											X"00000000",X"00000000",
											X"00000000",X"00000000",
											X"00000000",X"00000000",
											X"00000000",X"00000000",
											X"00000000",X"00000000",
											X"00000000",X"00000000");
											--ThirdLine $a0,$a1
begin
Process(RegWrite,CLK, ReadReg1,ReadReg2,WriteReg,WriteData,RegWrite)
begin 
ReadData1 <= RegArray(to_integer(unsigned(ReadReg1)));
ReadData2 <= RegArray(to_integer(unsigned(ReadReg2)));
if (rising_edge(CLK) and RegWrite = '1')
then 
RegArray(to_integer(unsigned(WriteReg)))<= WriteData ;
end if;
end process;
end Behavioral;

