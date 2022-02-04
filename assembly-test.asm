# Test for MIPS microprocessor

.data 0
num0: .word 1 # posic 0
num1: .word 2 # posic 4
num2: .word 4 # posic 8 
num3: .word 8 # posic 12 
num4: .word 16 # posic 16 
num5: .word 32 # posic 20
num6: .word 0 # posic 24
num7: .word 0 # posic 28
num8: .word 0 # posic 32
num9: .word 0 # posic 36
num10: .word 0 # posic 40
num11: .word 0 # posic 44
.text 0
main:
  # carga num0 a num5 en los registros 9 a 14
  lw $t1, 4($zero) # lw $r9, 4($r0)
  lw $t2, 4($zero) # lw $r10, 4($r0)
  lw $t3, 8($zero) # lw $r11, 8($r0)
  lw $t4, 12($zero) # lw $r12, 12($r0)
  lw $t5, 16($zero) # lw $r13, 16($r0)
  lw $t6, 20($zero) # lw $r14, 20($r0)
  nop
  nop
  nop
  nop

# Una instrucción tipo-R previa modifica un registro y este se usa en un
# branch. Hacer que el salto sea efectivo, y que sea no efectivo.
  add $t2, $t1, $t3 # en r10 un 6 = 2 + 4
  beq $t1, $t2, notomado1  # 2 == 6 ? Salto no efectivo
  add $t5, $t2, $t1  # en r13 8 = 6 + 2
  add $t6, $t3, $t4  # en  r14 12 = 8 + 4

notomado1:
  addi $t3, $t3, 4   # en r11 un 8 = 4 + 4
  beq $t3, $t4, tomado1  # 8 == 8? salto efectivo
  addi $t1, $zero, 6   # en r9 un 6
  addi $t2, $zero, 7   # en r10 un 7
  addi $t3, $zero, 8   # en r11 un 8

tomado1:

# Una lectura de memoria se guarda en un registro y a continuación se
# ejecuta un salto condiciona (branch). Hacer que el salto sea efectivo, y
# que sea no efectivo.
  lw $t5, 16($zero) # lw $r13, 16($r0)
  addi $t5, $t5, 16  # en r13 32 = 16 + 16
  lw $t6, 20($zero) # lw $r14, 20($r0)
  beq $t5, $t6, tomado2  # 32 == 32 ? salto efectivo
  addi $t1, $zero, 8   # en r9 un 8
  addi $t2, $zero, 9   # en r10 un 9
  addi $t3, $zero, 10   # en r11 un 10

tomado2:
  lw $t1, 4($zero) # lw $r9, 0($r0)
  lw $t2, 4($zero) # lw $r10, 4($r0)
  lw $t1, 0($zero) # lw $r9, 4($r0)
  beq $t1, $t2, notomado2  # 1 == 2 ? salto no efectivo
  addi $t3, $zero, 11   # en r12 un 11
  addi $t4, $zero, 12   # en r13 un 12
  addi $t5, $zero, 13   # en r14 un 13

notomado2:
  addi $t3, $zero, 7   # en r12 un 7
  addi $t4, $zero, 8   # en r13 un 8
  # lw tras beq
  beq $t3, $t4, fin  # 7 == 8 ? no efectivo
  lw $t4, 20($zero) # lw $r12, 20($r0)

fin:
  sw $t2, 0($zero) # Guarda el 2 en posicion 0 de memoria
  nop
  nop
  nop
