all: x86/myHelperBot.c
	gcc -o myHelperBot.x86 x86/myHelperBot.c

clean:
	rm myHelperBot.x86
