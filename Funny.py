
global money
money = 1000
global bet
bet = int(input("Enter your bet amount: "))
import random
def gambling(a, b, c):
    global money
    a = random.randint(1, 4)
    b = random.randint(1, 4)
    c = random.randint(1, 4)


    if a == b and b == c:
        print("\033[92mJackpot!")
        three = ["🍌🍌🍌", "🍋🍋🍋", "🍒🍒🍒", "🍉🍉🍉"]
        print(random.choice(three))
        print("+ $2500")
        money += 6 * bet
        print(money)
    elif a != b and b != c and a != c:
            print("\033[91mTry again!")
            one = ["🍌🍋🍒", "🍌🍋🍉", "🍌🍒🍋", "🍌🍒🍉", "🍌🍉🍋", "🍌🍉🍒",
"🍋🍌🍒", "🍋🍌🍉", "🍋🍒🍌", "🍋🍒🍉", "🍋🍉🍌", "🍋🍉🍒",
"🍒🍌🍋", "🍒🍌🍉", "🍒🍋🍌", "🍒🍋🍉", "🍒🍉🍌", "🍒🍉🍋",
"🍉🍌🍋", "🍉🍌🍒", "🍉🍋🍌", "🍉🍋🍒", "🍉🍒🍌", "🍉🍒🍋"]
            print(random.choice(one))
            print("you suck")
            money += 0
            print(money)
    else: 
        print("\033[93mSo close!")
        two = ["🍋🍌🍌", "🍌🍋🍌", "🍌🍌🍋", "🍒🍌🍌", "🍌🍒🍌", "🍌🍌🍒", "🍉🍌🍌",
            "🍌🍉🍌", "🍌🍌🍉", "🍌🍋🍋", "🍋🍌🍋", "🍋🍋🍌", "🍒🍋🍋", "🍋🍒🍋", 
            "🍋🍋🍒", "🍉🍋🍋", "🍋🍉🍋", "🍋🍋🍉", "🍌🍒🍒", "🍒🍌🍒", "🍒🍒🍌", 
            "🍋🍒🍒", "🍒🍋🍒", "🍒🍒🍋", "🍉🍒🍒", "🍒🍉🍒", "🍒🍒🍉", "🍌🍉🍉", 
            "🍉🍌🍉", "🍉🍉🍌", "🍋🍉🍉", "🍉🍋🍉", "🍉🍉🍋", "🍒🍉🍉", "🍉🍒🍉", "🍉🍉🍒"]
        print(random.choice(two))
        print("+ $100")
        money += 1.25 * bet
        print(money)
    return a, b, c
import tkinter as tk

root = tk.Tk()


class Button():
    def __init__(self, text, command):
        self.text = text
        self.command = command

        if self.command is None:
            self.command = self.button_clicked1

        self.button = tk.Button(root, 
                   text=self.text, 
                   command=self.command,
                   activebackground="blue", 
                   activeforeground="white",
                   anchor="center",
                   bd=3,
                   bg="lightgray",
                   cursor="hand2",
                   disabledforeground="gray",
                   fg="black",
                   font=("Arial", 12),
                   height=2,
                   highlightbackground="black",
                   highlightcolor="green",
                   highlightthickness=2,
                   justify="center",
                   overrelief="raised",
                   padx=10,
                   pady=5,
                   width=15,
                   wraplength=100)
          
    def button_clicked1(self):
        global money
        global bet
        gambling(0, 0, 0)
        money -= bet
        print(f"Remaining money: ${money}")

        self.changeText(f"Gamble ${bet}")
          
    def changeText(self, text):
        self.text = text
        self.button.configure(text=self.text)


# Creating a button with specified options
button = Button(f"Gamble ${bet}", None)

button.button.pack(padx=20, pady=20)


def button_clicked2():
    global money
    new_bet = int(input("Enter your new bet amount: "))
    global bet
    bet = new_bet
    print(f"Bet amount changed to: ${bet}")

    return bet

# Creating a button with specified options
button1 = Button("Change Bet", button_clicked2)

button1.button.pack(padx=20, pady=20)

root.mainloop()
