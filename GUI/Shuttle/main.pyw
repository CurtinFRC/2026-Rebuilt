import os
import sys
import tkinter as tk
import math
from networktables import NetworkTables


bintoggles = False
shoottoggles = False
didit = False 
amount = 0
AREANAx= -999999999999
AREANAy= -999999999999
NetworkTables.initialize(server='roborio-4788-frc.local')
sd = NetworkTables.getTable('SmartDashboard/ShuttlePos')
def load_asset(path):
    base = getattr(sys, '_MEIPASS', os.path.dirname(os.path.abspath(__file__)))
    assets = os.path.join(base, "assets")
    return os.path.join(assets, path)


jsonfile = load_asset("cords.json")

def place(event):
    global bintoggles, shoottoggles, amount, didit, AREANAx, AREANAy
    print(f"Current: {str(event.x)}, {str(event.y)}")
    if shoottoggles and event.y > 53:
        amount += 1
        if didit == True:
            canvas.delete("target")
        else:
            didit = True
        canvas.create_image(
            event.x,
            event.y,
            image=image_tagertt,
            tags=("target")
        )
        AREANAx = event.x  * 0.0332 
        AREANAy = (event.y - 52) * 0.0332  
        if AREANAx != -999999999999:
            sd.putNumber("X", AREANAx)
            sd.putNumber("Y", AREANAy)
    if not AREANAy == -999999999999:
        print(f"target: {str(AREANAx)}, {str(AREANAy)}")


def toggle(event):
    global shoottoggles, bintoggles
    shoottoggles = not shoottoggles
    bintoggles = False
    canvas.config(cursor="crosshair" if shoottoggles else "")


def bintoggle(event):
    global AREANAy, AREANAx
    canvas.delete("target")
    AREANAy = -999999999999
    AREANAx = -999999999999

window = tk.Tk()
window.geometry("491x292")
window.configure(bg="#242424")
window.title("FRC Target coordinator 2000")
window.iconbitmap(load_asset("icon2.ico"))
canvas = tk.Canvas(
    window,
    bg="#ffffff",
    width=491,
    height=292,
    bd=0,
    highlightthickness=0,
    relief="ridge"
)
canvas.place(x=0, y=0)

image_1 = tk.PhotoImage(file=load_asset("1.png"))
image_2 = tk.PhotoImage(file=load_asset("2.png"))
image_tagertt = tk.PhotoImage(file=load_asset("target.png"))
image_target = tk.PhotoImage(file=load_asset("targetbutton.png"))
image_bin = tk.PhotoImage(file=load_asset("binbutton.png"))

bg_img = canvas.create_image(245, 171, image=image_1)


gradient_width = 491
gradient_height = 52
gradient_image = tk.PhotoImage(width=gradient_width, height=gradient_height)
gradient_item = canvas.create_image(0, 0, anchor="nw", image=gradient_image)

offset = 0

def update_gradient():
    global offset

    pink = (255, 0, 120)
    yellow = (255, 230, 0)

    for x in range(gradient_width):
        ratio = (math.sin((x + offset) * 0.01) + 1) / 2

        r = int(pink[0] + (yellow[0] - pink[0]) * ratio)
        g = int(pink[1] + (yellow[1] - pink[1]) * ratio)
        b = int(pink[2] + (yellow[2] - pink[2]) * ratio)

        color = f"#{r:02x}{g:02x}{b:02x}"

        gradient_image.put(color, to=(x, 0, x+1, gradient_height))

    offset += 3
    window.after(30, update_gradient)

update_gradient()

canvas.create_image(75, 25, image=image_target, tag="shoottoggle")
canvas.create_image(120, 25, image=image_bin, tag="bintoggle")
canvas.create_image(28, 22, image=image_2)

canvas.bind("<Button-1>", place)
canvas.tag_bind("shoottoggle", "<Button-1>", toggle)
canvas.tag_bind("bintoggle", "<Button-1>", bintoggle)

window.resizable(False, False)
window.mainloop()
