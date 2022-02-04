--------------------------------------------------------------------------------
-- MIPS processor with pipeline
--
-- [AUTHOR]: David Pascual Hernández
--
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;

entity processor is
   port(
      Clk         : in  std_logic; -- Reloj activo en flanco subida
      Reset       : in  std_logic; -- Reset asincrono activo nivel alto
      -- Instruction memory
      IAddr      : out std_logic_vector(31 downto 0); -- Direccion Instr
      IDataIn    : in  std_logic_vector(31 downto 0); -- Instruccion leida
      -- Data memory
      DAddr      : out std_logic_vector(31 downto 0); -- Direccion
      DRdEn      : out std_logic;                     -- Habilitacion lectura
      DWrEn      : out std_logic;                     -- Habilitacion escritura
      DDataOut   : out std_logic_vector(31 downto 0); -- Dato escrito
      DDataIn    : in  std_logic_vector(31 downto 0)  -- Dato leido
   );
end processor;

architecture rtl of processor is

  component alu
    port(
      OpA : in std_logic_vector (31 downto 0);
      OpB : in std_logic_vector (31 downto 0);
      Control : in std_logic_vector (3 downto 0);
      Result : out std_logic_vector (31 downto 0);
      Zflag : out std_logic
    );
  end component;

  component reg_bank
     port (
        Clk   : in std_logic; -- Reloj activo en flanco de subida
        Reset : in std_logic; -- Reset as�ncrono a nivel alto
        A1    : in std_logic_vector(4 downto 0);   -- Direcci�n para el puerto Rd1
        Rd1   : out std_logic_vector(31 downto 0); -- Dato del puerto Rd1
        A2    : in std_logic_vector(4 downto 0);   -- Direcci�n para el puerto Rd2
        Rd2   : out std_logic_vector(31 downto 0); -- Dato del puerto Rd2
        A3    : in std_logic_vector(4 downto 0);   -- Direcci�n para el puerto Wd3
        Wd3   : in std_logic_vector(31 downto 0);  -- Dato de entrada Wd3
        We3   : in std_logic -- Habilitaci�n de la escritura de Wd3
     );
  end component reg_bank;

  component control_unit
     port (
        -- Entrada = codigo de operacion en la instruccion:
        OpCode   : in  std_logic_vector (5 downto 0);
        Func : in std_logic_vector (5 downto 0);  -- Bits de funcion (añadido)
        -- Seniales para el PC
        Branch   : out  std_logic; -- 1 = Ejecutandose instruccion branch
        Jump   : out  std_logic;  -- 1 = Ejecutandose instruccion jump
        -- Seniales relativas a la memoria
        MemToReg : out  std_logic; -- 1 = Escribir en registro la salida de la mem.
        MemWrite : out  std_logic; -- Escribir la memoria
        MemRead  : out  std_logic; -- Leer la memoria
        -- Seniales para la ALU
        ALUSrc   : out  std_logic;                     -- 0 = oper.B es registro, 1 = es valor inm.
        ALUOp    : out  std_logic_vector (2 downto 0); -- Tipo operacion para control de la ALU
        -- Seniales para el GPR
        RegWrite : out  std_logic; -- 1=Escribir registro
        RegDst   : out  std_logic  -- 0=Reg. destino es campo rt, 1=campo rd
     );
  end component;

  component alu_control is
   port (
      -- Entradas:
      ALUOp  : in std_logic_vector (2 downto 0); -- Codigo de control desde la unidad de control
      Funct  : in std_logic_vector (5 downto 0); -- Campo "funct" de la instruccion
      -- Salida de control para la ALU:
      ALUControl : out std_logic_vector (3 downto 0) -- Define operacion a ejecutar por la ALU
   );
  end component alu_control;
 
  -- Señales del cableado del procesador segmentado
  -- Señales etapa IF --
  signal IF_PCMuxJump		: std_logic_vector(31 downto 0); -- Dirección apuntada por la instrucción "Jump" anterior
  signal IF_PCMuxNoJump		: std_logic_vector(31 downto 0);
  signal IF_PCAddress		: std_logic_vector(31 downto 0);
  signal IF_InsMemIF		: std_logic_vector(31 downto 0); -- Valor de la instrucción actual en la etapa IF
  signal IF_PCNext		: std_logic_vector(31 downto 0);
  signal IFID_PCAddr4			: std_logic_vector(31 downto 0); -- Registro PC + 4 de IF/ID
  signal IFID_InsMemID		: std_logic_vector(31 downto 0); -- Valor de la instrucción actual en la etapa ID
  
  -- Señales etapa ID --
  signal IDEX_DataRead1		: std_logic_vector(31 downto 0); -- Registro rd1 leido en el banco de reg
  signal IDEX_DataRead2		: std_logic_vector(31 downto 0); -- Registro rd2 leido en el banco de reg
  signal IDEX_regRS  : std_logic_vector(4 downto 0);  -- Bits (25-21) de la instrucción actual en ID 
  signal IDEX_HighInstr		: std_logic_vector(4 downto 0); -- Bits (20-16) de la instrucción actual en ID
  signal IDEX_LowInstr		: std_logic_vector(4 downto 0); -- Bits (15-11) de la instrucción actual en ID  
  signal IDEX_ImmExtend		: std_logic_vector(31 downto 0); -- Salida del extensor de signo 16->32 bits en registro ID/EX
  signal ID_InmExt        : std_logic_vector(31 downto 0); -- La parte baja de la instrucción extendida de signo (var auxiliar)
  signal IDEX_PCAddr4			: std_logic_vector(31 downto 0); -- Registro PC + 4 de ID/EX
  signal ID_regRS, ID_regRT : std_logic_vector(31 downto 0); -- Señales auxiliares rd1 y rd2
  signal Ctrl_Jump, Ctrl_Branch, Ctrl_MemWrite, Ctrl_MemRead,  Ctrl_ALUSrc, Ctrl_RegDest, Ctrl_MemToReg, Ctrl_RegWrite : std_logic; -- Salidas auxiliares unidad de control
  signal Ctrl_ALUOP     : std_logic_vector(2 downto 0);  -- Salida auxliar AluOP unidad de control
  signal IDEX_JumpAddress  : std_logic_vector(25 downto 0);  -- Direccion de salto incondicional
  ---- Señales de control EX en ID/EX
  signal IDEX_ALUSrc, IDEX_RegDest  : std_logic;
  signal IDEX_ALUOP                 : std_logic_vector(2 downto 0); 
  ---- Señales de control M en ID/EX
  signal IDEX_Branch, IDEX_MemRead, IDEX_MemWrite, IDEX_Jump  : std_logic;
  ---- Señales de control WB en ID/EX
  signal IDEX_MemToReg, IDEX_RegWrite : std_logic;  
   
  -- Señales etapa EX --
  signal EXM_Branch, EXM_MemRead, EXM_MemWrite, EXM_Jump  : std_logic;  -- Señales de control M en EX/MEM
  signal EXM_MemToReg, EXM_RegWrite : std_logic; -- Señales de control WB en EX/MEM
  signal EX_AluControl   : std_logic_vector(3 downto 0); -- Señal interna salida de ALU control
  signal EX_regRD_aux       : std_logic_vector(4 downto 0);  -- Señal auxiliar salida MUX reg destino
  signal EXM_regRD : std_logic_vector(4 downto 0);  -- Señal EX/MEM salida MUX reg destino
  signal EX_AluOp2      : std_logic_vector(31 downto 0);  -- Señal auxiliar salida MUX operando 2 ALU
  signal EX_AluRes_aux        : std_logic_vector(31 downto 0);  -- Señal auxiliar salida ALU
  signal EXM_AluRes        : std_logic_vector(31 downto 0);  -- Señal EX/MEM salida ALU
  signal EX_ALUIgual_aux    : std_logic;  -- Señal auxiliar flag zero ALU
  signal EXM_ALUIgual    : std_logic;  -- Señal EX/MEM flag zero ALU
  signal EXM_DataRead2 : std_logic_vector(31 downto 0);  -- Señal EX/MEM: datos para escribir en la memoria
  signal EX_AddrBranch_aux    : std_logic_vector(31 downto 0);  -- Señal auxiliar salida suma direcciones branch
  signal EXM_AddrBranch    : std_logic_vector(31 downto 0);  -- Señal EX/MEM salida suma direcciones branch
  signal EX_AddrJump_aux  : std_logic_vector(31 downto 0);   -- Señal auxiliar salida suma direcciones jump
  signal EXM_AddrJump  : std_logic_vector(31 downto 0);   -- Señal EX/MEM salida suma direcciones jump
  signal EX_muxOp1 : std_logic_vector (31 downto 0);  -- Señal salida mux que escoge entre dato adelantado o dato del banco reg (op1)
  signal EX_muxOp2 : std_logic_vector (31 downto 0);  -- Señal salida mux que escoge entre dato adelantado o dato del banco reg (op2)
  
  -- Señales etapa MEM --
  signal MWB_MemToReg, MWB_RegWrite : std_logic; -- Señales de control WB en MEM/WB
  signal MEM_dataIn_Mem_aux     : std_logic_vector(31 downto 0); -- Dato leído auxiliar de la memoria de datos
  signal MWB_dataIn_Mem     : std_logic_vector(31 downto 0); -- Dato leído de la memoria de datos en MEM/WB
  signal MWB_regRD          : std_logic_vector(4 downto 0);  -- Señal MEM/WB reg destino
  signal MWB_RegDataDest         : std_logic_vector(31 downto 0);  -- Señal EX/MEM salida ALU registro destino

  -- Señales compartidas
  signal desition_Jump     : std_logic;  -- Flag de logica para el salto / pc+4
  signal Addr_Jump_dest : std_logic_vector(31 downto 0);  -- Direccion de salto
  signal reg_RD_data  : std_logic_vector(31 downto 0);  -- Datos a ser guardados en el banco de registros
  signal AND_Branch   : std_logic;  -- AND salto branch

  -- Fowarding Unit
  signal FowardA : std_logic_vector (1 downto 0);
  signal FowardB : std_logic_vector (1 downto 0);

  -- HazardUnit
  signal hazard_detected : std_logic;

begin

  -- Proceso IF
  IF_proc: process(Clk, Reset)
  begin
    if Reset = '1' then
      IF_PCAddress <= (others => '0');
      IFID_InsMemID <= (others => '0');
      IFID_PCAddr4 <= (others => '0');
    elsif rising_edge(Clk) then
      if hazard_detected = '0' or AND_Branch = '1' then  -- Riesgo de lw no existente o branch activado
        IFID_PCAddr4 <= IF_PCMuxNoJump;  -- Registro IF/ID de PC + 4
        IF_PCAddress <= IF_PCNext;   -- Actualizacion PC
        IFID_InsMemID <= IF_InsMemIF;  -- Registro IF/ID de la instruccion 32 bits
      else -- Hazard detectado, bubble de 1 ciclo
        IFID_PCAddr4 <= IFID_PCAddr4;  -- Registro IF/ID de PC + 4
        IF_PCAddress <= IF_PCAddress;   -- Actualizacion PC
        IFID_InsMemID <= IFID_InsMemID;
      end if;
    end if;
  end process;
  
  IF_PCMuxNoJump <= IF_PCAddress + 4;
  IF_PCMuxJump <= Addr_Jump_dest;
  IF_PCNext     <= IF_PCMuxJump when desition_Jump = '1' else IF_PCMuxNoJump;  -- mux actualizacion PC
  IAddr         <= IF_PCAddress;  -- Entrada memoria instrucciones
  IF_InsMemIF   <= IDataIn;  -- Salida memoria instrucciones
  
  RegsMIPS : reg_bank
  port map (
    Clk   => Clk,
    Reset => Reset,
    A1    => IFID_InsMemID(25 downto 21),
    Rd1   => ID_regRS,
    A2    => IFID_InsMemID(20 downto 16),
    Rd2   => ID_regRT,
    A3    => MWB_regRD, 
    Wd3   => reg_RD_data,
    We3   => MWB_RegWrite  
  );

  UnidadControl : control_unit
  port map(
    -- Señales de entrada de Control
    OpCode   => IFID_InsMemID(31 downto 26),
    Func   => IFID_InsMemID(5 downto 0),
    -- Señales para el PC
    Jump   => Ctrl_Jump,
    Branch   => Ctrl_Branch,
    -- Señales para la memoria
    MemToReg => Ctrl_MemToReg,
    MemWrite => Ctrl_MemWrite,
    MemRead  => Ctrl_MemRead,
    -- Señales para la ALU
    ALUSrc   => Ctrl_ALUSrc,
    ALUOP    => Ctrl_ALUOP,
    -- Señales para el GPR
    RegWrite => Ctrl_RegWrite,
    RegDst   => Ctrl_RegDest
  );

  -- Proceso ID
  ID_proc: process(Clk, Reset)
  begin
    if Reset = '1' then
      IDEX_DataRead1 <= (others => '0');
      IDEX_DataRead2 <= (others => '0');
      IDEX_HighInstr <= (others => '0');
      IDEX_LowInstr <= (others => '0');
      IDEX_ImmExtend <= (others => '0');
      IDEX_JumpAddress <= (others => '0');
      IDEX_regRS <= (others => '0');
      IDEX_PCAddr4 <= (others => '0');
      IDEX_ALUSrc <= '0';
      IDEX_RegDest <= '0';
      IDEX_ALUOP <= (others => '0');
      IDEX_Branch <= '0';
      IDEX_MemRead <= '0';
      IDEX_MemWrite <= '0';
      IDEX_Jump <= '0';
      IDEX_MemToReg <= '0';
      IDEX_RegWrite <=  '0';
    elsif rising_edge(Clk) then
      -- ID/EX: Salidas de la unidad de control
      if hazard_detected = '1' or AND_Branch = '1' then -- Data/Control Hazard detectado, generando instruccion NOP
        IDEX_DataRead1 <= (others => '0');
        IDEX_DataRead2 <= (others => '0');
        IDEX_HighInstr <= (others => '0');
        IDEX_LowInstr <= (others => '0');
        IDEX_ImmExtend <= (others => '0');
        IDEX_JumpAddress <= (others => '0');
        IDEX_regRS <= (others => '0');
        IDEX_PCAddr4 <= (others => '0');
        IDEX_ALUSrc <= '0';
        IDEX_RegDest <= '0';
        IDEX_ALUOP <= (others => '0');
        IDEX_Branch <= '0';
        IDEX_MemRead <= '0';
        IDEX_MemWrite <= '0';
        IDEX_Jump <= '0';
        IDEX_MemToReg <= '0';
        IDEX_RegWrite <=  '0';
      else 
        -- ID/EX: Salidas del banco de registros y datos de instruccion
        IDEX_DataRead1 <= ID_regRS;
        IDEX_DataRead2 <= ID_regRT;
        IDEX_HighInstr <= IFID_InsMemID(20 downto 16);
        IDEX_LowInstr <= IFID_InsMemID(15 downto 11);
        IDEX_ImmExtend <= ID_InmExt;
        IDEX_JumpAddress <= IFID_InsMemID(25 downto 0);
        IDEX_regRS <= IFID_InsMemID(25 downto 21);
        -- ID/EX: PC+4
        IDEX_PCAddr4 <= IFID_PCAddr4;
        -- Control
        IDEX_ALUSrc <= Ctrl_ALUSrc;
        IDEX_RegDest <= Ctrl_RegDest;
        IDEX_ALUOP <= Ctrl_ALUOP;
        IDEX_Branch <= Ctrl_Branch;
        IDEX_MemRead <= Ctrl_MemRead;
        IDEX_MemWrite <= Ctrl_MemWrite;
        IDEX_Jump <= Ctrl_Jump;
        IDEX_MemToReg <= Ctrl_MemToReg;
        IDEX_RegWrite <= Ctrl_RegWrite;
      end if;
    end if;
  end process;

  ID_InmExt        <= x"FFFF" & IFID_InsMemID(15 downto 0) when IFID_InsMemID(15)='1' else   -- Operacion ext de signo 16 -> 32 bits
                    x"0000" & IFID_InsMemID(15 downto 0);

  -- Hazard detection unit
  hazard_detected <= '1' when IDEX_MemRead = '1' and (IDEX_HighInstr = IFID_InsMemID(25 downto 21) or IDEX_HighInstr = IFID_InsMemID(20 downto 16)) else
                     '0';

  Alu_control_i: alu_control
  port map(
    -- Entradas:
    ALUOp  => IDEX_ALUOP, -- Codigo de control desde la unidad de control
    Funct  => IDEX_ImmExtend (5 downto 0), -- Campo "funct" de la instruccion
    -- Salida de control para la ALU:
    ALUControl => EX_AluControl  -- Define operacion a ejecutar por la ALU
  );

  Alu_MIPS : alu
  port map (
    OpA     => EX_muxOp1,
    OpB     => EX_AluOp2,
    Control => EX_AluControl,
    Result  => EX_AluRes_aux,
    Zflag   => EX_ALUIgual_aux
  );

  -- Proceso EX
  EX_proc: process(Clk, Reset)
  begin
    if Reset = '1' then
      EXM_Branch <= '0';
      EXM_MemRead <= '0';
      EXM_MemWrite <= '0';
      EXM_Jump <= '0';
      EXM_MemToReg <= '0';
      EXM_RegWrite <= '0';
      EXM_regRD <= (others => '0');
      EXM_AluRes <= (others => '0');
      EXM_ALUIgual <= '0';
      EXM_DataRead2 <= (others => '0');
      EXM_AddrBranch <= (others => '0');
      EXM_AddrJump <= (others => '0');  
    elsif rising_edge(Clk) then
      if AND_Branch = '0' then
        -- EX/MEM: Señales de control WB y M
        EXM_Branch <= IDEX_Branch;
        EXM_MemRead <= IDEX_MemRead;
        EXM_MemWrite <= IDEX_MemWrite;
        EXM_Jump <= IDEX_Jump;
        EXM_MemToReg <= IDEX_MemToReg;
        EXM_RegWrite <= IDEX_RegWrite;
        -- EX/MEM: registro destino
        EXM_regRD <= EX_regRD_aux;
        -- EX/MEM: Salida ALU
        EXM_AluRes <= EX_AluRes_aux;
        EXM_ALUIgual <= EX_ALUIgual_aux;
        -- EX/MEM: Datos a escribir en memoria
        EXM_DataRead2 <= EX_muxOp2;
        -- EX/MEM: Direcciones de salto branch y J
        EXM_AddrBranch <= EX_AddrBranch_aux;
        EXM_AddrJump <= EX_AddrJump_aux; 
      else  -- Control hazard detectado, generando NOP
        EXM_Branch <= '0';
        EXM_MemRead <= '0';
        EXM_MemWrite <= '0';
        EXM_Jump <= '0';
        EXM_MemToReg <= '0';
        EXM_RegWrite <= '0';
        EXM_regRD <= (others => '0');
        EXM_AluRes <= (others => '0');
        EXM_ALUIgual <= '0';
        EXM_DataRead2 <= (others => '0');
        EXM_AddrBranch <= (others => '0');
        EXM_AddrJump <= (others => '0');  
      end if;
    end if;
  end process;

  EX_regRD_aux   <= IDEX_HighInstr when IDEX_RegDest = '0' else IDEX_LowInstr;  -- Mux etapa EX: eleccion bits del registro destino
  EX_AluOp2      <= EX_muxOp2 when IDEX_ALUSrc = '0' else IDEX_ImmExtend;       -- Mux etapa EX: seleccion operando 2 ALU

  EX_AddrBranch_aux    <= IDEX_PCAddr4 + ( IDEX_ImmExtend(29 downto 0) & "00");  -- Suma + shift left de direcciones Branch
  EX_AddrJump_aux      <= IDEX_PCAddr4(31 downto 28) & IDEX_JumpAddress & "00";  -- Suma + shift left de direccion Jump incondicional

  -- Fowarding Unit
  FowardA   <=  "01" when MWB_RegWrite = '1' and 
                          MWB_regRD /= "00000" and
                          MWB_regRD = IDEX_regRS and
                          EXM_regRD /= IDEX_regRS else
                "10" when EXM_RegWrite = '1' and 
                          EXM_regRD /= "00000" and  
                          EXM_regRD = IDEX_regRS else
                "00";

  FowardB   <=  "01" when MWB_RegWrite = '1' and 
                          MWB_regRD /= "00000" and
                          MWB_regRD = IDEX_HighInstr and
                          EXM_regRD /= IDEX_HighInstr else
                "10" when EXM_RegWrite = '1' and 
                          EXM_regRD /= "00000" and
                          EXM_regRD = IDEX_HighInstr else
                "00";
  
  -- Multiplexores que seleccionan dato segun codigo generado por Fowarding Unit
  EX_muxOp1 <= IDEX_DataRead1 when FowardA = "00" else -- Dato leido del campo de registros
               EXM_AluRes when FowardA = "10" else     -- Resultado anticipado EX/MEM
               reg_RD_data when FowardA = "01";        -- Resultado anticipado MEM/WB
  EX_muxOp2 <= IDEX_DataRead2 when FowardB = "00" else -- Dato leido del campo de registros
               EXM_AluRes when FowardB = "10" else     -- Resultado anticipado EX/MEM
               reg_RD_data when FowardB = "01";        -- Resultado anticipado MEM/WB

  -- Proceso MEM
  MEM_proc: process(Clk, Reset)
  begin
    if Reset = '1' then
      MWB_MemToReg <= '0'; 
      MWB_RegWrite <= '0'; 
      MWB_regRD <= (others => '0'); 
      MWB_RegDataDest <= (others => '0'); 
      MWB_dataIn_Mem <= (others => '0');
    elsif rising_edge(Clk) then
      -- MEM/WB: Señales de control WB
      MWB_MemToReg <= EXM_MemToReg;
      MWB_RegWrite <= EXM_RegWrite;
      -- MEM/WB: registro destino
      MWB_regRD <= EXM_regRD;
      -- MEM/WB: dato resultado ALU para reg destino
      MWB_RegDataDest <= EXM_AluRes;
      -- MEM/WB: dato leido en la memoria de datos
      MWB_dataIn_Mem <= MEM_dataIn_Mem_aux;
    end if;
  end process;

  -- Entrada/salida memoria de datos
  DAddr      <= EXM_AluRes;
  DDataOut   <= EXM_DataRead2;
  DWrEn      <= EXM_MemWrite;
  dRdEn      <= EXM_MemRead;
  MEM_dataIn_Mem_aux <= DDataIn;

  -- Regs_eq_branch <= '1' when (regRS = regRT) else '0'; -- Si no funciona el flag de ALU usar esto
 
  -- Logica de salto
  AND_Branch <= EXM_Branch and EXM_ALUIgual;
  desition_Jump  <= EXM_Jump or AND_Branch;  -- Si esta a 1, la proxima instruccion sera en salto y no pc+4
  Addr_Jump_dest <= EXM_AddrJump   when EXM_Jump='1' else       -- Mux decision salto incondicional o branch
                    EXM_AddrBranch when EXM_Branch='1' else
                    (others =>'0');

  -- (ETAPA WB) Mux Datos a guardar en el banco de registros 
  reg_RD_data <= MWB_dataIn_Mem when MWB_MemToReg = '1' else MWB_RegDataDest;

end architecture;
