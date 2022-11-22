----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date:    11:11:33 03/14/2022 
-- Design Name: 
-- Module Name:    Control - Behavioral 
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
-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx primitives in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity Control is
    Port ( OP : in  STD_LOGIC_VECTOR (5 downto 0);
           RegDst : out  STD_LOGIC;
           ALUSrc : out  STD_LOGIC;
           MemtoReg : out  STD_LOGIC;
           RegWrite : out  STD_LOGIC;
           MemRead : out  STD_LOGIC;
           MemWrite : out  STD_LOGIC;
           Branch : out  STD_LOGIC;
           ALUOP0 : out  STD_LOGIC;
			  ALUOP1 : out  STD_LOGIC);
end Control;

architecture Behavioral of Control is

begin
PROCESS(OP)
BEGIN
IF OP = "000000" THEN RegDst <= '1'; ALUSrc <= '0';MemtoReg <= '0';RegWrite <='1' ;MemRead <= '0' ;MemWrite <= '0' ;Branch <= '0' ;ALUOP0 <= '1';ALUOP1 <= '0';
ELSIF OP = "100011" THEN RegDst <= '0'; ALUSrc <= '1';MemtoReg <= '1';RegWrite <='1' ;MemRead <= '1' ;MemWrite <= '0' ;Branch <= '0' ;ALUOP0 <= '0';ALUOP1 <= '0';
ELSIF OP = "101011" THEN ALUSrc <= '1';RegWrite <='0' ;MemRead <= '0' ;MemWrite <= '1' ;Branch <= '0' ;ALUOP0 <= '0';ALUOP1 <= '0';
ELSIF OP = "000100" THEN ALUSrc <= '0';RegWrite <='0' ;MemRead <= '0' ;MemWrite <= '0' ;Branch <= '1' ;ALUOP0 <= '0';ALUOP1 <= '1';

END IF;
END PROCESS;

--	process(OP) 
--		begin 
--			case OP is
--				when "000000"=>
--					REGDST<='1';
--					ALUSRC<='0';
--					MEMTOREG<='0';
--					REGWRITE<='1';
--					MEMREAD<='0';
--					MEMWRITE<='0';
--					BRANCH<='0';
--					ALUOP1<='1';
--					ALUOP0<='0';
--				when "100011"=>
--					REGDST<='0';
--					ALUSRC<='1';
--					MEMTOREG<='1';
--					REGWRITE<='1';
--					MEMREAD<='1';
--					MEMWRITE<='0';
--					BRANCH<='0';
--					ALUOP1<='0';
--					ALUOP0<='0'; 
--				when "101011"=>
--					REGDST<='0';
--					ALUSRC<='1';
--					MEMTOREG<='0';
--					REGWRITE<='0';
--					MEMREAD<='0';
--					MEMWRITE<='1';
--					BRANCH<='0';
--					ALUOP1<='0';
--					ALUOP0<='0';	
--				when "000100"=>
--					REGDST<='0';
--					ALUSRC<='0';
--					MEMTOREG<='0';
--					REGWRITE<='0';
--					MEMREAD<='0';
--					MEMWRITE<='0';
--					BRANCH<='1';
--					ALUOP1<='0';
--					ALUOP0<='1';
--					
--				when others=>NULL; 
--				   REGDST<='1';
--					ALUSRC<='0';
--					MEMTOREG<='0';
--					REGWRITE<='1';
--					MEMREAD<='0';
--					MEMWRITE<='0';
--					BRANCH<='0';
--					ALUOP1<='1';
--					ALUOP0<='0';
--					
--	 end case;
--	end process ;
end Behavioral;

