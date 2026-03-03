package org.firstinspires.ftc.teamcode.util;

import com.google.gson.Gson;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Scanner;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;

public class FileConfig {
    private static final Gson gson = new Gson();
    public static void writeFile(String filename, String contents) {
        File writeFile = AppUtil.getInstance().getSettingsFile(filename);
        ReadWriteFile.writeFile(writeFile, contents);
    }
    public static String readFile(String filename) {
        File readFile = AppUtil.getInstance().getSettingsFile(filename);
        StringBuilder sb = new StringBuilder();
        try (Scanner scan = new Scanner(readFile)) {
            while (scan.hasNextLine()) {
                sb.append(scan.nextLine());
            }
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }
        return sb.toString();
    }

    public static void saveJson(String filename, Object data) {
        String json = gson.toJson(data);
        writeFile(filename, json);
    }
    public static <T> T loadJson(String filename, Class<T> clazz) {
        String json = readFile(filename);
        if (json == null || json.isEmpty()) return null;
        return gson.fromJson(json,clazz);
    }
    public static void saveStaticClass(String filename, Class<?> clazz) {
        try {
            Object tempInstance = clazz.newInstance();

            for (Field field : clazz.getDeclaredFields()) {
                if (!Modifier.isStatic(field.getModifiers())) continue;
                field.setAccessible(true);
                Object value = field.get(null);
                field.set(tempInstance, value);
            }
            saveJson(filename, tempInstance);
        }
        catch (Exception e) {
            e.printStackTrace();
        }
    }
    public static void loadStaticClass(String filename, Class<?> clazz) {
        try {
            Object loaded = loadJson(filename, clazz);
            if (loaded == null) return;

            for (Field field : clazz.getDeclaredFields()) {
                if (!Modifier.isStatic(field.getModifiers())) continue;

                field.setAccessible(true);
                Object value = field.get(loaded);
                field.set(null,value);
            }
        }
        catch (Exception e) {
            e.printStackTrace();
        }
    }
}

