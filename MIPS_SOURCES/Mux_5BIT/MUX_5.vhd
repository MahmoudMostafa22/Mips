----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date:    18:08:45 05/18/2022 
-- Design Name: 
-- Module Name:    MUX_5 - Behavioral 
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
--use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx primitives in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity MUX_5 is
    Port ( A : in  STD_LOGIC_VECTOR (4 downto 0);
           B : in  STD_LOGIC_VECTOR (4 downto 0);
           OUTM : out  STD_LOGIC_VECTOR (4 downto 0);
           S : in  STD_LOGIC);
end MUX_5;

architecture Behavioral of MUX_5 is

begin
process(A,B,S)
begin 
if (S = '0') then OUTM <= A;
elsif (S = '1') then OUTM <= B;
end if ;
end process;


end Behavioral;

