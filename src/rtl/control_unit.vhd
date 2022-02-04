--------------------------------------------------------------------------------
-- Main control unit.
--
-- [AUTHOR]: David Pascual Hernández
--
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;

entity control_unit is
   port (
      -- Entrada = codigo de operacion en la instruccion:
      OpCode  : in  std_logic_vector (5 downto 0);
      Func : in std_logic_vector (5 downto 0);  -- Bits de funcion (añadido)
      -- Seniales para el PC
      Branch : out  std_logic; -- 1 = Ejecutandose instruccion branch
      Jump : out std_logic; -- 1 = Salto incondicional (añadido)
      -- Seniales relativas a la memoria
      MemToReg : out  std_logic; -- 1 = Escribir en registro la salida de la mem.
      MemWrite : out  std_logic; -- Escribir la memoria
      MemRead  : out  std_logic; -- Leer la memoria
      -- Seniales para la ALU
      ALUSrc : out  std_logic;                     -- 0 = oper.B es registro, 1 = es valor inm.
      ALUOp  : out  std_logic_vector (2 downto 0); -- Tipo operacion para control de la ALU
      -- Seniales para el GPR
      RegWrite : out  std_logic; -- 1=Escribir registro
      RegDst   : out  std_logic  -- 0=Reg. destino es rt, 1=rd      
   );
end control_unit;

architecture rtl of control_unit is
   -- Tipo para los codigos de operacion:
   subtype t_opCode is std_logic_vector (5 downto 0);
   -- Codigos de operacion para las diferentes instrucciones:
   constant OP_RTYPE  : t_opCode := "000000";
   constant OP_BEQ    : t_opCode := "000100";
   constant OP_SW     : t_opCode := "101011";
   constant OP_LW     : t_opCode := "100011";
   constant OP_LUI    : t_opCode := "001111";
   constant OP_ADDI    : t_opCode := "001000"; -- añadido
   constant OP_SLTI    : t_opCode := "001010"; -- añadido
   constant OP_J      : t_opCode := "000010"; -- añadido

begin
--   OPCode <= Instr(31 downto 26);

Branch   <= '1' when opCode = OP_BEQ else '0';

ALUSrc   <= '0' when opCode = OP_RTYPE else -- R-type
            '0' when opCode = OP_BEQ   else -- beq
            '0' when opCode = OP_J   else -- beq
            '1'; -- lw, sw -- (tambien addi y slti)

RegDst   <= '1' when opCode = OP_RTYPE and Func /= "000000" else -- R-type (no NOP)
            '0'; -- lw, sw, beq, j, I-type -- (tambien addi y slti)

RegWrite <= '0' when opCode = OP_SW  else -- sw
            '0' when opCode = OP_BEQ else -- beq
            '0' when opCode = OP_J else -- j
            '0' when opCode = OP_RTYPE and Func = "000000" else -- Nop (añadido)
            '1'; -- R-type, lw, I-type, jal -- (tambien addi y slti)

MemRead  <= '1' when opCode = OP_LW else --lw
            '0';

MemWrite <= '1' when opCode = OP_SW else -- sw
            '0'; 

MemToReg <= '1' when opCode = OP_LW else -- lw
            '0'; -- R-type, sw, beq, j, jal, I-type -- (tambien addi y slti)

ALUOP    <= "000" when opCode = OP_LW  else -- lw
            "000" when opCode = OP_SW  else -- sw
            "000" when opCode = OP_ADDI  else -- addi (añadido)
            "001" when opCode = OP_BEQ else -- beq
            "011" when opCode = OP_LUI else -- lui
            "100" when opCode = OP_SLTI else -- slti (añadido, codigo incluido)
            "000" when opCode = OP_J else -- J (añadido)
            "000" when opCode = OP_RTYPE and Func = "000000" else -- Nop (añadido)
            "010"; -- r-type;

Jump <= '1' when opCode = "000010" else -- j
        '1' when opCode = "000011" else --jal
        '0';

end architecture;
