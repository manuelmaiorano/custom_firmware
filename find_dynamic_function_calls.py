
#-----------------------------------------------------------------
# pycparser: func_calls.py
#
# Using pycparser for printing out all the calls of some function
# in a C file.
#
# Eli Bendersky [https://eli.thegreenplace.net/]
# License: BSD
#-----------------------------------------------------------------
import sys


from pycparser import c_ast, parse_file, CParser


INC = r"-I. -I./include -I./fake_libc_include/ -I./Third_Party/FreeRTOS/Source/include -I./Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I./vendor/libdw1000/inc -I./Drivers/STM32F7xx_HAL_Driver/Inc -I./Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I./Drivers/CMSIS/Device/ST/STM32F7xx -I./Drivers/CMSIS/Device/ST/STM32F7xx/Include -I./Third_Party/SEGGER -I./CMSIS-DSP/Include -I./CMSIS-DSP/PrivateInclude -I./Drivers/MPU6050/interface -I./Drivers/MPU6050/example -I./Drivers/MPU6050/src"
INC = INC.split(" ")
DEF = "-DSTM32F767xx -DUSE_HAL_DRIVER -DUSE_FULL_ASSERT=1 -D__IO="" -D__I="""
DEF = DEF.split(" ")
ARGS = INC + DEF

def delete_preprocess(text: str):
    return "\n".join(list(filter(lambda x: not x.startswith("#"), text.split("\n"))))

dynfunc2func = {}
staticfundyncalls = {}

# A visitor with some state information (the funcname it's looking for)
class FuncCallVisitor(c_ast.NodeVisitor):
    def __init__(self, caller):
        self.caller = caller

    def visit_FuncCall(self, node):
        if hasattr(node.name, 'type') and node.name.type == "->":
            structure = node.name.name.name
            function = node.name.field.name
            dynfunc2func[function] = ""
            staticfundyncalls[self.caller] = function
            print(f"{structure}->{function}")
        # Visit args in case they contain more func calls.
        if node.args:
            self.visit(node.args)


class FuncDefVisitor(c_ast.NodeVisitor):
    def visit_FuncDef(self, node):
        v = FuncCallVisitor(node.decl.name)

        v.visit(node)

class AssignmentVisitor(c_ast.NodeVisitor):
    def visit_Assignment(self, node):
        print(node)

class DeclVisitor(c_ast.NodeVisitor):
    def visit_Decl(self, node):
        print(node)

class TypedefVisitor(c_ast.NodeVisitor):
    def visit_Typedef(self, node):
        print(node)

class InitListVisitor(c_ast.NodeVisitor):
    def visit_InitList(self, node):
        for expr in node.exprs:
            if hasattr(expr, "name") and expr.name[0].name in dynfunc2func:
                dynfunc2func[expr.name[0].name] = expr.expr.name

def show_dyn_func_calls(filename):
    ast = parse_file(filename, use_cpp=True, cpp_args=ARGS)
    
    v = FuncDefVisitor()
    v.visit(ast)

def find_matching(filename):
    ast = parse_file(filename, use_cpp=True, cpp_args=ARGS)

    v = InitListVisitor()
    v.visit(ast)


if __name__ == "__main__":
    filenames = ['src/locodeck.c', 'vendor/libdw1000/src/libdw1000Spi.c', 'vendor/libdw1000/src/libdw1000.c', 'src/lpsTdoa2Tag.c']
    
    for filename in filenames:
        show_dyn_func_calls(filename)

    for filename in filenames:
        find_matching(filename)

    print(dynfunc2func)
    print(staticfundyncalls)