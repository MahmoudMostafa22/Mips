----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date:    21:46:09 05/06/2022 
-- Design Name: 
-- Module Name:    InstMem - Behavioral 
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
use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values


-- Uncomment the following library declaration if instantiating
-- any Xilinx primitives in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity InstMem is
    Port ( Address : in  STD_LOGIC_VECTOR (31 downto 0);
           Instruction : out  STD_LOGIC_VECTOR (31 downto 0);
			  CLK : in STD_LOGIC);
end InstMem;

architecture Behavioral of InstMem is
type INSTArray is array (0 to 31) of STD_LOGIC_VECTOR (7 downto 0);
signal IM : INSTArray:= (
								x"00",x"85",x"10",x"20",	--add $v0, $a0, $a1
								x"AC",x"02",x"00",x"08",	--sw $v0, 8($zero)
								x"8C",x"06",x"00",x"08",	--lw $a2, 8($zero)
								x"10",x"46",x"00",x"01",	--beq $v0, $a2, Good_Processor
								x"00",x"46",x"88",x"2A",	--slt $s1, $v0, $a2
								x"00",x"A4",x"88",x"22",	--Good_Processor: sub $s1, $a1, $a0
								x"00",x"00",x"00",x"00",
								x"00",x"00",x"00",x"00");

begin

Instruction(31 downto 24) <= IM(to_integer(unsigned(Address)));
Instruction(23 downto 16) <= IM(to_integer(unsigned(Address))+1);
Instruction(15 downto 8) <= IM(to_integer(unsigned(Address))+2);
Instruction(7 downto 0) <= IM(to_integer(unsigned(Address))+3);

end Behavioral;

