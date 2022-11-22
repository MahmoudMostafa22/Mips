----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date:    10:55:48 02/28/2022 
-- Design Name: 
-- Module Name:    ALU4Bit - Behavioral 
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
use IEEE.STD_LOGIC_ARITH.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;

entity ALU4Bit is
    Port ( A : in  STD_LOGIC_VECTOR (31 downto 0);
           B : in  STD_LOGIC_VECTOR (31 downto 0);
           ALUin : in  STD_LOGIC_VECTOR (3 downto 0);
           zero : out  STD_LOGIC;
           result : out  STD_LOGIC_VECTOR (31 downto 0));
end ALU4Bit;

architecture Behavioral of ALU4Bit is

begin
process(A,B,ALUin)
begin 
	if ALUin = "0000"
		then result <= A and B ;
	elsif ALUin = "0001"
		then result <= A or B;
	elsif ALUin = "0111"
		then if  A < B then result <= X"00000001";
		else result <= X"00000000";
		end if;
	elsif ALUin = "0010"
		then result <= A + B;
	elsif ALUin = "0110"
		then result <= A - B;
	elsif ALUin = "1100"
		then result <= A nor B;
	end if ;
	if A = B
	then zero <= '1';
	else zero <= '0';
	end if ;
	end PROCESS;

end Behavioral;

