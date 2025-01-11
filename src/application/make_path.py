import tkinter as tk
from tkinter import filedialog
import json
import xml.etree.ElementTree as ET

# Main application class
class PathDrawer:
    def __init__(self, root):
        self.root = root
        self.root.title("Path Drawer")

        self.canvas = tk.Canvas(root, width=800, height=600, bg="white")
        self.canvas.pack()

        self.points = []  # List to store points
        self.canvas.bind("<Button-1>", self.add_point)  # Left click to add point
        self.canvas.bind("<Button-3>", self.reset_canvas)  # Right click to reset

        # Buttons for saving
        save_json_btn = tk.Button(root, text="Save as JSON", command=self.save_as_json)
        save_json_btn.pack(side=tk.LEFT, padx=5, pady=5)

        save_xml_btn = tk.Button(root, text="Save as XML", command=self.save_as_xml)
        save_xml_btn.pack(side=tk.LEFT, padx=5, pady=5)

    def add_point(self, event):
        """Add a point and draw a line to the previous point."""
        x, y = event.x, event.y
        self.points.append((x, y))

        if len(self.points) > 1:
            self.canvas.create_line(self.points[-2], self.points[-1], fill="black", width=2)

        self.canvas.create_oval(x-3, y-3, x+3, y+3, fill="red")

    def reset_canvas(self, event=None):
        """Reset the canvas and clear all points."""
        self.canvas.delete("all")
        self.points = []

    def save_as_json(self):
        """Save the points as a JSON file."""
        if not self.points:
            print("No points to save!")
            return

        file_path = filedialog.asksaveasfilename(defaultextension=".json", filetypes=[("JSON files", "*.json")])
        if file_path:
            with open(file_path, "w") as f:
                json.dump({"points": self.points}, f, indent=4)
            print(f"Saved points to {file_path}")

    def save_as_xml(self):
        """Save the points as an XML file."""
        if not self.points:
            print("No points to save!")
            return

        file_path = filedialog.asksaveasfilename(defaultextension=".xml", filetypes=[("XML files", "*.xml")])
        if file_path:
            root = ET.Element("Path")
            for i, (x, y) in enumerate(self.points):
                point = ET.SubElement(root, "Point", id=str(i))
                ET.SubElement(point, "X").text = str(x)
                ET.SubElement(point, "Y").text = str(y)

            tree = ET.ElementTree(root)
            tree.write(file_path, encoding="utf-8", xml_declaration=True)
            print(f"Saved points to {file_path}")

if __name__ == "__main__":
    root = tk.Tk()
    app = PathDrawer(root)
    root.mainloop()
