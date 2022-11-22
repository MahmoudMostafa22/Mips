----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date:    16:09:01 05/06/2022 
-- Design Name: 
-- Module Name:    Adder4Bit - Behavioral 
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
USE IEEE.STD_LOGIC_ARITH.ALL;
USE IEEE.STD_LOGIC_UNSIGNED.ALL;



entity Adder4Bit is
    Port ( INPUT1 : in  STD_LOGIC_VECTOR (31 downto 0);
           OUTPUT1 : out  STD_LOGIC_VECTOR (31 downto 0));
end Adder4Bit;

architecture Behavioral of Adder4Bit is

begin
process(INPUT1)
begin 
OUTPUT1 <= INPUT1 + X"00000004";
end process;

end Behavioral;

