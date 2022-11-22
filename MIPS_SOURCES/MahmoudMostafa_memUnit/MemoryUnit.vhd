----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date:    10:54:10 03/28/2022 
-- Design Name: 
-- Module Name:    MemoryUnit - Behavioral 
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

entity MemoryUnit is
    Port ( MemRead : in  STD_LOGIC;
           MemWrite : in  STD_LOGIC;
           Address : in  STD_LOGIC_VECTOR (31 downto 0);
           DataWrite : in  STD_LOGIC_VECTOR (31 downto 0);
           DataRead : out  STD_LOGIC_VECTOR (31 downto 0);
           CLK : in  STD_LOGIC);
end MemoryUnit;

architecture Behavioral of MemoryUnit is
type MEMArray is array (0 to 35) of STD_LOGIC_VECTOR (7 downto 0);
signal RAM: MEMArray:= (x"AB",x"CD",x"EF",x"00",
								x"75",x"74",x"65",x"72",
								x"20",x"41",x"72",x"63",
								x"68",x"69",x"74",x"65",
								x"12",x"34",x"56",x"78",
								x"7F",x"7F",x"6D",x"6D",
								x"00",x"00",x"00",x"00",
								x"78",x"78",x"6A",x"6A",
								x"00",x"00",x"00",x"01");
								
begin
process(MemRead,MemWrite,Address,CLK)
begin
if (MemRead = '1' and MemWrite ='0')
then DataRead(31 downto 24) <= RAM(to_integer(unsigned(Address)));
DataRead(23 downto 16) <= RAM(to_integer(unsigned(Address)+1));
DataRead(15 downto 8) <= RAM(to_integer(unsigned(Address)+2));
DataRead(7 downto 0) <= RAM(to_integer(unsigned(Address)+3));
elsif (MemRead = '0' and MemWrite ='1' and rising_edge(CLK))
then RAM(to_integer(unsigned(Address))) <= DataWrite(31 downto 24);
RAM(to_integer(unsigned(Address)+1)) <= DataWrite(23 downto 16);
RAM(to_integer(unsigned(Address)+2)) <= DataWrite(15 downto 8);
RAM(to_integer(unsigned(Address)+3)) <= DataWrite(7 downto 0);
end if ;
end process;
end Behavioral;

