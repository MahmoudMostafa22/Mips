----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date:    22:33:48 05/02/2022 
-- Design Name: 
-- Module Name:    Mux2_1 - Behavioral 
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

entity Mux2_1 is
    Port ( IN1 : in  STD_LOGIC_VECTOR (31 downto 0);
           IN2 : in  STD_LOGIC_VECTOR (31 downto 0);
           S : in  STD_LOGIC;
           OUTMUX : out  STD_LOGIC_VECTOR (31 downto 0));
end Mux2_1;

architecture Behavioral of Mux2_1 is
begin
process (IN1,IN2,S)
begin 
if (S = '0') then OUTMUX <= IN1;
elsif (S = '1') then OUTMUX <= IN2;
end if ;
end process;

end Behavioral;

