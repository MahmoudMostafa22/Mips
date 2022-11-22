--------------------------------------------------------------------------------
-- Company: 
-- Engineer:
--
-- Create Date:   22:37:01 05/02/2022
-- Design Name:   
-- Module Name:   D:/Documents/Year 3/Term 6/Xilinx Programs/Computer Architecture/Mux/testestt.vhd
-- Project Name:  Mux
-- Target Device:  
-- Tool versions:  
-- Description:   
-- 
-- VHDL Test Bench Created by ISE for module: Mux2_1
-- 
-- Dependencies:
-- 
-- Revision:
-- Revision 0.01 - File Created
-- Additional Comments:
--
-- Notes: 
-- This testbench has been automatically generated using types std_logic and
-- std_logic_vector for the ports of the unit under test.  Xilinx recommends
-- that these types always be used for the top-level I/O of a design in order
-- to guarantee that the testbench will bind correctly to the post-implementation 
-- simulation model.
--------------------------------------------------------------------------------
LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
 
-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--USE ieee.numeric_std.ALL;
 
ENTITY testestt IS
END testestt;
 
ARCHITECTURE behavior OF testestt IS 
 
    -- Component Declaration for the Unit Under Test (UUT)
 
    COMPONENT Mux2_1
    PORT(
         IN1 : IN  std_logic_vector(31 downto 0);
         IN2 : IN  std_logic_vector(31 downto 0);
         S : IN  std_logic;
         OUTMUX : OUT  std_logic_vector(31 downto 0)
        );
    END COMPONENT;
    

   --Inputs
   signal IN1 : std_logic_vector(31 downto 0) := (others => '0');
   signal IN2 : std_logic_vector(31 downto 0) := (others => '0');
   signal S : std_logic := '0';

 	--Outputs
   signal OUTMUX : std_logic_vector(31 downto 0);
   -- No clocks detected in port list. Replace <clock> below with 
   -- appropriate port name 
 
 
BEGIN
 
	-- Instantiate the Unit Under Test (UUT)
   uut: Mux2_1 PORT MAP (
          IN1 => IN1,
          IN2 => IN2,
          S => S,
          OUTMUX => OUTMUX
        );

   -- Clock process definitions


   -- Stimulus process
   stim_proc: process
   begin		
      -- hold reset state for 100 ns.
     IN1 <= X"00000000" ;IN2 <= X"00000001"; S <= '0'; wait for 100 ns;
	  IN1 <= X"00012000" ;IN2 <= X"00000001"; S <= '1'; wait for 100 ns;
	  IN1 <= X"00036000" ;IN2 <= X"00000011"; S <= '0'; wait for 100 ns;
	  IN1 <= X"00000022" ;IN2 <= X"00000601"; S <= '1'; wait for 100 ns;
      -- insert stimulus here 
   end process;

END;
