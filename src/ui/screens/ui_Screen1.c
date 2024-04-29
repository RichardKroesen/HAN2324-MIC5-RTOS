// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.4.0
// LVGL version: 8.3.11
// Project name: Oog 2 - robotachtig

#include "../ui.h"

void ui_Screen1_screen_init(void)
{
    ui_Screen1 = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_Screen1, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Image10 = lv_img_create(ui_Screen1);
    lv_img_set_src(ui_Image10, &ui_img_achtergrond_oog_2_png);
    lv_obj_set_width(ui_Image10, LV_SIZE_CONTENT);   /// 347
    lv_obj_set_height(ui_Image10, LV_SIZE_CONTENT);    /// 347
    lv_obj_set_align(ui_Image10, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Image10, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_Image10, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_img_set_zoom(ui_Image10, 700);

    ui_Image2 = lv_img_create(ui_Screen1);
    lv_img_set_src(ui_Image2, &ui_img_iris_png);
    lv_obj_set_width(ui_Image2, LV_SIZE_CONTENT);   /// 236
    lv_obj_set_height(ui_Image2, LV_SIZE_CONTENT);    /// 237
    lv_obj_set_align(ui_Image2, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Image2, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_Image2, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Image3 = lv_img_create(ui_Screen1);
    lv_img_set_src(ui_Image3, &ui_img_puntjes_png);
    lv_obj_set_width(ui_Image3, LV_SIZE_CONTENT);   /// 236
    lv_obj_set_height(ui_Image3, LV_SIZE_CONTENT);    /// 237
    lv_obj_set_align(ui_Image3, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Image3, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_Image3, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Image5 = lv_img_create(ui_Screen1);
    lv_img_set_src(ui_Image5, &ui_img_pupil_png);
    lv_obj_set_width(ui_Image5, LV_SIZE_CONTENT);   /// 236
    lv_obj_set_height(ui_Image5, LV_SIZE_CONTENT);    /// 237
    lv_obj_set_align(ui_Image5, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Image5, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_Image5, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Image6 = lv_img_create(ui_Screen1);
    lv_img_set_src(ui_Image6, &ui_img_glansjes_png);
    lv_obj_set_width(ui_Image6, LV_SIZE_CONTENT);   /// 236
    lv_obj_set_height(ui_Image6, LV_SIZE_CONTENT);    /// 237
    lv_obj_set_align(ui_Image6, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Image6, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_Image6, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

}