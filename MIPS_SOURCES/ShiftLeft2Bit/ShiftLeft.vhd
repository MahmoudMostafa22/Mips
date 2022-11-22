----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date:    10:53:25 04/04/2022 
-- Design Name: 
-- Module Name:    ShiftLeft - Behavioral 
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

entity ShiftLeft is
    Port ( Inshift : in  STD_LOGIC_VECTOR (31 downto 0);
           Outshift : out  STD_LOGIC_VECTOR (31 downto 0));
end ShiftLeft;

architecture Behavioral of ShiftLeft is

begin

process(Inshift)
begin
Outshift(1 downto 0) <= "00";
Outshift(31 downto 2) <= Inshift(29 downto 0); 
end process;

end Behavioral;

