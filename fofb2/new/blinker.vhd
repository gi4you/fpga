----------------------------------------------------------------------------------
--
----------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;

entity blinker is
  generic (
    	GEN_CLK_FREQUENCY : natural := 100000000 ; 	-- 100MHz
		GEN_BLINK_PERIOD  : real    := 0.1			-- 10 Hz
  ) ;
  port (
    	P_I_CLK : in std_logic ;
	 	P_O_LED : out std_logic
  ) ;
end blinker;

architecture behavioral of blinker is
  signal SIG_LED : std_logic := '0' ;
begin
  P_O_LED <= SIG_LED ;
  
  process(P_I_CLK)
	 variable v_cnt : natural ;
  begin
	 if P_I_CLK'event and P_I_CLK = '1' then
	   if v_cnt < integer(real(GEN_CLK_FREQUENCY)*GEN_BLINK_PERIOD)/2 then
	     v_cnt := v_cnt + 1 ;
	   else
	     v_cnt := 0 ;
	     SIG_LED <= not SIG_LED ;
		end if ;
	 end if ;
  end process ;

end behavioral;


