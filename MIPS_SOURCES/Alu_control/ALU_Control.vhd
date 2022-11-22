----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date:    08:07:58 03/25/2022 
-- Design Name: 
-- Module Name:    ALU_Control - Behavioral 
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

entity ALU_Control is
    Port ( ALUOP : in  STD_LOGIC_VECTOR (1 downto 0);
           FuncField : in  STD_LOGIC_VECTOR (5 downto 0);
           Operation : out  STD_LOGIC_VECTOR (3 downto 0));
end ALU_Control;

architecture Behavioral of ALU_Control is

begin
process(ALUOP,FuncField)
begin
if (ALUOP = "00") then Operation <= "0010";
elsif (ALUOP = "01") then Operation <= "0110";
elsif ((ALUOP = "10") and ((FuncField(3)='0')and(FuncField(2)='0') and (FuncField(1)='0')and (FuncField(0)='0'))) then Operation <= "0010";
elsif ((ALUOP = "10") and ((FuncField(3)='0')and(FuncField(2)='1') and (FuncField(1)='0')and (FuncField(0)='0'))) then Operation <= "0000";
elsif ((ALUOP = "10") and ((FuncField(3)='0')and(FuncField(2)='1') and (FuncField(1)='0')and (FuncField(0)='1'))) then Operation <= "0001";
elsif ((ALUOP(1) = '1') and ((FuncField(3)='0')and(FuncField(2)='0') and (FuncField(1)='1')and (FuncField(0)='0'))) then Operation <= "0110";
elsif ((ALUOP(1) = '1') and ((FuncField(3)='1')and(FuncField(2)='0') and (FuncField(1)='1')and (FuncField(0)='0'))) then Operation <= "0111";
end if ;
end process;
end Behavioral;

