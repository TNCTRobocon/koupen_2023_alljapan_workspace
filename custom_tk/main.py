import json
import tkinter as tk
from tkinter import messagebox
from tkinter import colorchooser
import customtkinter as ctk
from functools import partial


class App(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.title("CTk widget preview")
        self.geometry("1200x780")
        self.minsize(1000,740)
        # default color settings
        ctk.set_appearance_mode("dark")
        ctk.set_default_color_theme("blue")
        # set grid layout 1x2
        self.grid_rowconfigure(0, weight=1)
        self.grid_columnconfigure(1, weight=1)

        # navigation menu lists
        self.navi_list = ["CTkFrame", "CTkButton", "CTkEntry", "CTkLabel", "CTkComboBox", "CTkCheckBox", "CTkRadioButton", "CTkProgressBar",
            "CTkOptionMenu", "CTkSwitch", "CTkSlider", "CTkScrollbar", "CTkScrollableFrame", "CTkSegmentedButton", "CTkTextbox", "CTkTabview"]
        # font settings
        self.title_font = ctk.CTkFont(family='Yu Gothic', size=26, weight='bold')
        self.navi1_font = ctk.CTkFont(family='Yu Gothic', size=16, weight='bold')
        self.navi2_font = ctk.CTkFont(family='Yu Gothic', size=14, weight='bold')
        self.normal_font = ctk.CTkFont(family='Yu Gothic', size=13)

        # create navigation frame
        self.create_navigation_frame()
        # create main frame
        self.create_mainframe()
        # Create sample widget
        self.create_example_widget()
        try:
            # json file loading
            self.color = json.load(open('./initial_theme.json', 'r'))
        except FileNotFoundError:
            messagebox.showinfo('error', 'initial_theme.json does not exist.\nPlease prepare the json file for the theme needed to display the configuration items.')
        # create setting widget
        self.create_setting_widget()
        # Insert Entry Values
        self.insert_entry_values('all')
        # select default frame
        self.select_frame_by_name(f"{self.navi_list[0]}")


    # create navigation frame
    def create_navigation_frame(self):
        self.navi_frame = ctk.CTkFrame(self, corner_radius=0)
        self.navi_frame.grid(row=0, column=0, sticky="nsew")
        self.navi_frame.grid_rowconfigure(len(self.navi_list)+1, weight=1)
        # Application Title
        self.navi_label = ctk.CTkLabel(self.navi_frame, text="■ Widget Preview", font=self.navi1_font)
        self.navi_label.grid(row=0, column=0, padx=(10,20), pady=20)
        # navigation menu
        self.navi_button = [ctk.CTkButton(self.navi_frame, corner_radius=0, height=38, border_spacing=6, text=f" {self.navi_list[i]} >",
                                          fg_color="transparent", text_color=("gray10", "gray90"), hover_color=("gray70", "gray30"),
                                          anchor="w", font=self.navi2_font, command=partial(self.select_frame_by_name, f'{self.navi_list[i]}'))
                            for i in range(len(self.navi_list))]
        for i in range(len(self.navi_list)):
            self.navi_button[i].grid(row=i+1, column=0, sticky="ew")
        # Appearance mode selection
        self.appearance_mode_menu = ctk.CTkOptionMenu(self.navi_frame, values=["Light", "Dark", "System"],
                                                        command=self.change_appearance_mode_event)
        self.appearance_mode_menu.grid(row=len(self.navi_list)+1, column=0, padx=20, pady=20, sticky="s")

    def change_appearance_mode_event(self, new_appearance_mode):
        ctk.set_appearance_mode(new_appearance_mode)

    # select navi frame
    def select_frame_by_name(self, name):
        # set button color for selected button
        for i in range(len(self.navi_list)):
            self.navi_button[i].configure(fg_color=("gray75", "gray25") if name == f"{self.navi_list[i]}" else "transparent")
        # show selected frame
        for i in range(len(self.navi_list)):
            if name == f"{self.navi_list[i]}":
                self.main_frame[i].grid(row=0, column=1, sticky="nsew")
            else:
                self.main_frame[i].grid_forget()

    # create main frame
    def create_mainframe(self):
        self.main_frame = [ctk.CTkFrame(self, corner_radius=0, fg_color="transparent") for _ in range(len(self.navi_list))]
        for i in range(len(self.navi_list)):
            self.main_frame[i].grid(row=0, column=0, sticky="nsew")
            self.main_frame[i].grid_rowconfigure(0, weight=1)
            self.main_frame[i].grid_columnconfigure(1, weight=1)
        # Create sub frame
        self.sub_frame = [[ctk.CTkFrame(self.main_frame[i], corner_radius=0, fg_color="transparent") for _ in range(2)] for i in range(len(self.navi_list))]
        for i in range(len(self.navi_list)):
            self.sub_frame[i][0].grid(row=0, column=0, padx=20, pady=(10, 20), sticky="nsew")
            self.sub_frame[i][1].grid(row=0, column=1, padx=20, pady=30, sticky="nsew")
            self.sub_frame[i][1].configure(border_width=2, border_color="gray50", corner_radius=10)
            for j in range(2):
                self.sub_frame[i][j].grid_rowconfigure(0, weight=1)
                self.sub_frame[i][j].grid_columnconfigure(0, weight=1)

    # create sample widget in right frame
    def create_example_widget(self):
        self.example = []
        self.example.append(ctk.CTkFrame(self.sub_frame[0][1]))
        self.example.append(ctk.CTkButton(self.sub_frame[1][1]))
        self.example.append(ctk.CTkEntry(self.sub_frame[2][1]))
        self.example.append(ctk.CTkLabel(self.sub_frame[3][1]))
        self.example.append(ctk.CTkComboBox(self.sub_frame[4][1]))
        self.example.append(ctk.CTkCheckBox(self.sub_frame[5][1]))
        self.example.append(ctk.CTkRadioButton(self.sub_frame[6][1]))
        self.example.append(ctk.CTkProgressBar(self.sub_frame[7][1]))
        self.example.append(ctk.CTkOptionMenu(self.sub_frame[8][1]))
        self.example.append(ctk.CTkSwitch(self.sub_frame[9][1]))
        self.example.append(ctk.CTkSlider(self.sub_frame[10][1]))
        self.example.append(ctk.CTkScrollbar(self.sub_frame[11][1]))
        self.example.append(ctk.CTkScrollableFrame(self.sub_frame[12][1]))
        self.example.append(ctk.CTkSegmentedButton(self.sub_frame[13][1]))
        self.example.append(ctk.CTkTextbox(self.sub_frame[14][1]))
        self.example.append(ctk.CTkTabview(self.sub_frame[15][1]))
        for i, item in enumerate(self.navi_list):
            self.example[i].grid(row=0, column=0, padx=20, pady=10)
            if item == 'CTkTabview':
                self.example[i].add("Tab 1")
                self.example[i].add("Tab 2")
        self.example[14].insert('0.0', 'sample_text, '*50)

    # create setting widgets
    def create_setting_widget(self):
        # create widget list
        self.example_frame = []
        self.example_title = []
        self.example_label = []
        self.example_entry = []
        self.example_codes = []
        self.example_apply = []
        self.example_reset = []
        self.color_hexnum = []
        self.color_button = []
        # create setting
        for i, item in enumerate(self.navi_list):
            # list keys in dict
            keys = list(self.color.get(item).keys())
            # create setting widget frame
            self.example_frame.append(ctk.CTkFrame(self.sub_frame[i][0], fg_color="transparent"))
            self.example_frame[i].grid(row=0, column=0, padx=20, pady=(0,20), sticky="new")
            self.example_frame[i].grid_columnconfigure(0, weight=1)
            # create setting widget
            self.example_title.append(ctk.CTkLabel(self.example_frame[i], text=f"{item} Settings",
                                                   corner_radius=10, height=60, width=400, font=self.title_font,
                                                   fg_color=["#3a7ebf", "#1f538d"], text_color=["#DCE4EE", "#DCE4EE"]))
            self.example_label.append([ctk.CTkLabel(self.example_frame[i], text=f"{key}", width=150,
                                                    wraplength=150, anchor='e', justify='right', font=self.normal_font)
                                       for key in keys])
            self.example_entry.append([[ctk.CTkEntry(self.example_frame[i], font=self.normal_font) for _ in range(2)]
                                       if 'color' in key else [ctk.CTkEntry(self.example_frame[i], font=self.normal_font)]
                                       for key in keys])
            self.example_codes.append(ctk.CTkEntry(self.example_frame[i], font=self.normal_font))
            self.example_apply.append(ctk.CTkButton(self.example_frame[i], text="Apply to settings", font=self.normal_font,
                                                    command=partial(self.apply_buttuon_click, i)))
            self.example_reset.append(ctk.CTkButton(self.example_frame[i], text="Reset preview", font=self.normal_font,
                                                    command=partial(self.preview_reset, i)))
            self.color_hexnum.append(ctk.CTkEntry(self.example_frame[i], font=self.normal_font))
            self.color_button.append(ctk.CTkButton(self.example_frame[i], text="color chooser", font=self.normal_font,
                                                   command=partial(self.color_choose_tool, i)))
            # widgets put in frame
            self.example_title[i].grid(row=0, column=0, columnspan=3, padx=20, pady=20, sticky="nsew")
            for j, key in enumerate(keys):
                self.example_label[i][j].grid(row=j+1, column=0, padx=10, pady=3, sticky="nsew")
                self.example_entry[i][j][0].grid(row=j+1, column=1, padx=10, pady=3, sticky="nsew")
                if 'color' in key:
                    self.example_entry[i][j][1].grid(row=j+1, column=2, padx=10, pady=3, sticky="nsew")
            self.example_codes[i].grid(row=j+2, column=0, columnspan=3, padx=10, pady=(10, 2), sticky="nsew")
            self.example_apply[i].grid(row=j+3, column=0, columnspan=2, padx=(10, 2), pady=(2, 10), sticky="nsew")
            self.example_reset[i].grid(row=j+3, column=2, padx=(2, 10), pady=(2, 10), sticky="nsew")
            self.color_hexnum[i].grid(row=j+4, column=1, padx=(10, 2), pady=4, sticky="nsew")
            self.color_button[i].grid(row=j+4, column=2, padx=(2, 10), pady=4, sticky="nsew")

    def color_choose_tool(self, i):
        selected_color = colorchooser.askcolor()
        self.color_hexnum[i].insert(0, selected_color[1])

    def insert_entry_values(self, i):
        def insert_value():
            keys = list(self.color.get(self.navi_list[i]).keys())
            values = list([self.color[self.navi_list[i]][key] for key in keys])
            for j, key in enumerate(keys):
                self.example_entry[i][j][0].delete(0, tk.END)
                if values[j] != None:
                    if 'color' in key:
                        if values[j] == 'transparent':
                            self.example_entry[i][j][0].insert(0, 'transparent')
                        else:
                            self.example_entry[i][j][1].delete(0, tk.END)
                            [self.example_entry[i][j][k].insert(0, f"{values[j][k]}") for k in range(2)]
                    else:
                        self.example_entry[i][j][0].insert(0, f"{values[j]}")
        if i == 'all':
            for i in range(len(self.navi_list)):
                insert_value()
        else:
            insert_value()

    def preview_reset(self, i):
        self.example[i].grid_forget()
        self.example_codes[i].delete(0, tk.END)
        self.create_example_widget()
        self.insert_entry_values(i)

    # Apply to preview widget & create example code
    def apply_buttuon_click(self, i):
        keys, values = self.get_widgets_data(i)
        self.create_example_code(i, keys, values)
        self.apply_example_configure(i, keys, values)

    # Collect configuration items entered in setting widgets
    def get_widgets_data(self, i):
        keys = list(self.color.get(self.navi_list[i]).keys())
        values = []
        for j, key in enumerate(keys):
            value = self.example_entry[i][j][0].get()
            if value == '':
                values.append(None)
            else:
                # color setting('transparent' or color list)
                if 'color' in key:
                    if value == 'transparent':
                        values.append('transparent')
                    elif self.example_entry[i][j][1].get() == '':
                        values.append(self.example_entry[i][j][0].get())
                    else:
                        values.append([self.example_entry[i][j][k].get() for k in range(2)])
                # font setting(tuple)
                elif key == 'font' or key == 'dropdown_font' or key == 'label_font':
                    for replace_word in ['(', ')', '"', "'", ' ']:
                        value = value.replace(replace_word, '')
                    values.append(tuple([int(ele) if k == 1 else ele for k, ele in enumerate(value.split(','))]))
                # values setting(list)
                elif key == 'values':
                    for replace_word in ['"', "'"]:
                        value = value.replace(replace_word, '')
                    values.append([ele for ele in value.split(',')])
                # int or str
                else:
                    try:
                        values.append(int(value))
                    except ValueError:
                        values.append(value)
        return keys, values

    def create_example_code(self, i, keys, values):
        excample_code = f'customtkinter.{self.navi_list[i]}(self, '
        for j, key in enumerate(keys):
            if values[j] != None:
                if isinstance(values[j], str):
                    excample_code += f'{key}="{values[j]}", '
                else:
                    excample_code += f'{key}={values[j]}, '
        excample_code = excample_code[:-2] + ')'
        self.example_codes[i].delete(0, tk.END)
        self.example_codes[i].insert(0, excample_code)

    # Apply to preview widget
    def apply_example_configure(self, i, keys, values):
        for j, key in enumerate(keys):
            if 'color' in key:
                if values[j] != None:
                    if key == 'fg_color':
                        self.example[i].configure(fg_color=values[j])
                    elif key == 'top_fg_color':
                        self.example[i].configure(top_fg_color=values[j])
                    elif key == 'border_color':
                        self.example[i].configure(border_color=values[j])
                    elif key == 'hover_color':
                        self.example[i].configure(hover_color=values[j])
                    elif key == 'progress_color':
                        self.example[i].configure(progress_color=values[j])
                    elif key == 'button_color':
                        self.example[i].configure(button_color=values[j])
                    elif key == 'button_hover_color':
                        self.example[i].configure(button_hover_color=values[j])
                    elif key == 'selected_color':
                        self.example[i].configure(selected_color=values[j])
                    elif key == 'selected_hover_color':
                        self.example[i].configure(selected_hover_color=values[j])
                    elif key == 'unselected_color':
                        self.example[i].configure(unselected_color=values[j])
                    elif key == 'unselected_hover_color':
                        self.example[i].configure(unselected_hover_color=values[j])
                    elif key == 'scrollbar_button_color':
                        self.example[i].configure(scrollbar_button_color=values[j])
                    elif key == 'scrollbar_button_hover_color':
                        self.example[i].configure(scrollbar_button_hover_color=values[j])
                    elif key == 'label_text_color':
                        self.example[i].configure(label_text_color=values[j])
                    elif key == 'label_fg_color':
                        self.example[i].configure(label_fg_color=values[j])
                    elif key == 'segmented_button_fg_color':
                        self.example[i].configure(segmented_button_fg_color=values[j])
                    elif key == 'segmented_button_selected_color':
                        self.example[i].configure(segmented_button_selected_color=values[j])
                    elif key == 'segmented_button_selected_hover_color':
                        self.example[i].configure(segmented_button_selected_hover_color=values[j])
                    elif key == 'segmented_button_unselected_color':
                        self.example[i].configure(segmented_button_unselected_color=values[j])
                    elif key == 'segmented_button_unselected_hover_color':
                        self.example[i].configure(segmented_button_unselected_hover_color=values[j])
                    elif key == 'dropdown_fg_color':
                        self.example[i].configure(dropdown_fg_color=values[j])
                    elif key == 'dropdown_hover_color':
                        self.example[i].configure(dropdown_hover_color=values[j])
                    elif key == 'dropdown_text_color':
                        self.example[i].configure(dropdown_text_color=values[j])
                    elif key == 'placeholder_text_color':
                        self.example[i].configure(placeholder_text_color=values[j])
                    elif key == 'text_color':
                        try:
                            self.example[i].configure(text_color=values[j])
                        except ValueError:
                            pass
                    elif key == 'text_color_disabled':
                        try:
                            self.example[i].configure(text_color_disabled=values[j])
                        except ValueError:
                            pass
            elif key == 'text':
                self.example[i].configure(text=values[j])
            elif key == 'label_text':
                self.example[i].configure(label_text=values[j])
            elif key == 'placeholder_text':
                self.example[i].configure(placeholder_text=values[j])
            elif key == 'number_of_steps':
                self.example[i].configure(number_of_steps=values[j])
            elif key == 'values':
                if values[j] != None:
                    self.example[i].configure(values=values[j])
            elif key == 'font':
                if values[j] != None:
                    self.example[i].configure(font=values[j])
            elif key == 'dropdown_font':
                if values[j] != None:
                    self.example[i].configure(dropdown_font=values[j])
            elif key == 'label_font':
                if values[j] != None:
                    self.example[i].configure(label_font=values[j])
            elif key == 'from_':
                if values[j] != None:
                    self.example[i].configure(from_=values[j])
            elif key == 'to':
                if values[j] != None:
                    self.example[i].configure(to=values[j])
            elif key == 'width':
                default_value = [200, 140, 140, 0, 140, 100, 100, 200, 140, 100, 200, 16, 200, 140, 200, 300]
                self.example[i].configure(width=default_value[i] if values[j] == None else values[j])
            elif key == 'height':
                default_value = [200, 28, 28, 28, 28, 24, 22, 8, 28, 24, 16, 200, 200, 28, 200, 250]
                self.example[i].configure(height=default_value[i] if values[j] == None else values[j])
            elif key == 'corner_radius':
                default_value = [6, 6, 6, 0, 6, 6, 1000, 1000, 6, 1000, 1000, 1000, 6, 6, 6, 6]
                self.example[i].configure(corner_radius=default_value[i] if values[j] == None else values[j])
            elif key == 'wraplength':
                self.example[i].configure(wraplength=0 if values[j] == None else values[j])
            elif key == 'border_width':
                default_value = [0, 0, 2, '', 2, 3, '', 0, '', 3, 6, '', 0, 3, 0, 0]
                self.example[i].configure(border_width=default_value[i] if values[j] == None else values[j])
            elif key == 'border_spacing':
                default_value = ['', 2, '', '', '', '', '', '', '', '', '', 4, '', '', 3, '']
                self.example[i].configure(border_spacing=default_value[i] if values[j] == None else values[j])
            elif key == 'hover':
                self.example[i].configure(hover=True if values[j] == None else values[j])
            elif key == 'radiobutton_width':
                self.example[i].configure(radiobutton_width=22 if values[j] == None else values[j])
            elif key == 'radiobutton_height':
                self.example[i].configure(radiobutton_height=22 if values[j] == None else values[j])
            elif key == 'checkbox_width':
                self.example[i].configure(checkbox_width=24 if values[j] == None else values[j])
            elif key == 'checkbox_height':
                self.example[i].configure(checkbox_height=24 if values[j] == None else values[j])
            elif key == 'switch_width':
                self.example[i].configure(switch_width=36 if values[j] == None else values[j])
            elif key == 'switch_height':
                self.example[i].configure(switch_height=18 if values[j] == None else values[j])
            elif key == 'button_length':
                self.example[i].configure(button_length=0 if values[j] == None else values[j])
            elif key == 'border_width_checked':
                self.example[i].configure(border_width_checked=6 if values[j] == None else values[j])
            elif key == 'border_width_unchecked':
                self.example[i].configure(border_width_unchecked=3 if values[j] == None else values[j])
            elif key == 'button_corner_radius':
                self.example[i].configure(button_corner_radius=1000 if values[j] == None else values[j])
            elif key == 'padx':
                self.example[i].configure(padx=0 if values[j] == None else values[j])
            elif key == 'pady':
                self.example[i].configure(pady=0 if values[j] == None else values[j])
            elif key == 'anchor':
                if values[j] in ['center','n', 'ne', 'e', 'se', 's', 'sw', 'w', 'nw']:
                    self.example[i].configure(anchor=values[j])
                elif values[j] == None:
                    self.example[i].configure(anchor='center')
            elif key == 'label_anchor':
                if values[j] in ['center','n', 'ne', 'e', 'se', 's', 'sw', 'w', 'nw']:
                    self.example[i].configure(label_anchor=values[j])
                elif values[j] == None:
                    self.example[i].configure(label_anchor='center')
            elif key == 'justify':
                if values[j] in ['center','right', 'left']:
                    self.example[i].configure(justify=values[j])
                elif values[j] == None:
                    self.example[i].configure(justify='left')
            elif key == 'mode':
                if values[j] in ['determinate', 'indeterminate']:
                    self.example[i].configure(mode=values[j])
                elif values[j] == None:
                    self.example[i].configure(mode='determinate')
            elif key == 'wrap':
                if values[j] in ['char','word', 'none']:
                    self.example[i].configure(wrap=values[j])
                elif values[j] == None:
                    self.example[i].configure(wrap='char')
            elif key == 'orientation':
                try:
                    if values[j] in ['horizontal', 'vertical']:
                        self.example[i].configure(orientation=values[j])
                    elif values[j] == None:
                        self.example[i].configure(orientation='horizontal')
                except ValueError:
                    pass

def main():
    app = App()
    app.mainloop()

if __name__ == "__main__":
    main()

