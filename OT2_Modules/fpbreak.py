
def work():
    print(" \033[0;37;41m Starting FP break    \033[0m")
    foo = 5
    boo = foo - 5
    foo = foo / boo # divide by zero here
    print(" \033[0;37;41m Not what I expected to happen      \033[0m")
