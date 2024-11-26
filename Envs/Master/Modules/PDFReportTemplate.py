#!/usr/bin/env python
# -*- coding:utf-8 -*-
# @FileName: ReportGenerator.py
# @Time: 2/6/23 11:09 AM
# @Author: Bu Yujun

import os
import shutil
import sys

from PIL import Image
from reportlab.graphics.shapes import Drawing  # 绘图工具
from reportlab.graphics.shapes import Image as DrawingImage
from reportlab.lib import colors  # 颜色模块
from reportlab.lib.enums import TA_CENTER, TA_LEFT
from reportlab.lib.pagesizes import landscape, A4  # 页面的标志尺寸(8.5*inch, 11*inch)
from reportlab.lib.styles import ParagraphStyle  # 文本样式
from reportlab.pdfbase import pdfmetrics  # 注册字体
from reportlab.pdfbase.ttfonts import TTFont  # 字体类
from reportlab.platypus import SimpleDocTemplate, PageBreak
from reportlab.platypus import Spacer, Paragraph, Table, TableStyle

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from Libs import get_project_path

# 注册字体(提前准备好字体文件, 如果同一个文件需要多种字体可以注册多个)
project_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
pdfmetrics.registerFont(TTFont('YaHei', os.path.join(get_project_path(), 'Docs/Resources/fonts/msyh.ttc')))
pdfmetrics.registerFont(TTFont('YaHeiB', os.path.join(get_project_path(), 'Docs/Resources/fonts/msyhbd.ttc')))
PAGE_HEIGHT = landscape(A4)[1]
PAGE_WIDTH = landscape(A4)[0]


class PDFReportTemplate:

    def __init__(self, report_title, test_time, tester, version, title_page, logo, title_summary_img=None):
        self.rightMargin = 30
        self.leftMargin = 30
        self.topMargin = 35
        self.bottomMargin = 30
        self.report_title = report_title
        self.test_time = test_time
        self.title_page = title_page
        self.title_summary_img = title_summary_img
        self.version = version
        self.logo = logo
        self.base_report_path = f'{report_title}.pdf'
        self.organization = '仿真开发与软件测试'
        self.tester = tester
        self.project_path = get_project_path()
        self.sequence_title_style = ParagraphStyle(
            name='sequence_title_style',
            alignment=TA_CENTER,
            fontName='YaHeiB',
            fontSize=34,
            textColor=colors.steelblue,
        )
        self.case_title_style = ParagraphStyle(
            name='case_title_style',
            alignment=TA_CENTER,
            fontName='YaHeiB',
            fontSize=34,
            textColor=colors.seagreen,
        )
        self.sub_title_style = ParagraphStyle(
            name='title_style',
            alignment=TA_LEFT,
            fontName='YaHeiB',
            leftIndent=PAGE_WIDTH / 2,
            fontSize=20,
            textColor=colors.lightsteelblue,
        )
        self.sub_title_style_1 = ParagraphStyle(
            name='title_style',
            alignment=TA_LEFT,
            fontName='YaHeiB',
            leftIndent=PAGE_WIDTH / 2,
            fontSize=16,
            textColor=colors.darkred,
        )
        self.stamp_style = ParagraphStyle(
            name='stamp_style',
            alignment=TA_CENTER,
            fontName='YaHeiB',
            fontSize=15,
            textColor=colors.grey,
        )
        self.heading_style = ParagraphStyle(
            name='heading_style',
            alignment=0,
            fontName='YaHeiB',
            fontSize=20,
            textColor=colors.darkred,
        )
        self.content_style = ParagraphStyle(
            name='content_style',
            alignment=0,
            fontName='YaHei',
            fontSize=11,
            textColor=colors.black,
        )
        self.page_count = 0
        self.content = []
        self.current_page_num = 0

    def FirstPage(self, canvas, report):
        canvas.saveState()

        canvas.drawImage(self.title_page, 0, 80, PAGE_WIDTH, PAGE_WIDTH / 2.03)
        
        if self.title_summary_img:
            width, height = Image.open(self.title_summary_img).size
            r = 3.2
            canvas.drawImage(self.title_summary_img,
                             PAGE_WIDTH / 2 - width / r / 2,
                             280,
                             width / r, height / r)

        canvas.setFillColor(colors.darkred)
        canvas.setFont('YaHeiB', 36)
        canvas.drawCentredString(PAGE_WIDTH / 2, 490, self.report_title)

        canvas.setFillColor(colors.darkgreen)
        canvas.setFont('YaHei', 15)
        canvas.drawCentredString(PAGE_WIDTH / 2, 460, f'{self.version}')

        canvas.setFillColor(colors.black)
        canvas.setFont('YaHei', 15)
        canvas.drawCentredString(PAGE_WIDTH / 2, 435, f'{self.organization} {self.tester}')
        canvas.setFont('YaHei', 15)
        canvas.drawCentredString(PAGE_WIDTH / 2, 410, self.test_time)

        self.PageInfo(canvas)
        canvas.restoreState()

    def LaterPages(self, canvas, report):
        canvas.saveState()
        self.PageInfo(canvas)
        canvas.restoreState()

    def PageInfo(self, canvas):
        canvas.setStrokeColor(colors.dimgrey)
        canvas.line(30, PAGE_HEIGHT - 30, PAGE_WIDTH - 170, PAGE_HEIGHT - 30)
        img = self.logo
        canvas.drawImage(img, PAGE_WIDTH - 150, PAGE_HEIGHT - 45, 135, 30)
        canvas.line(30, 40, PAGE_WIDTH - 30, 40)
        canvas.setFont('YaHei', 9)
        canvas.setFillColor(colors.black)
        canvas.drawString(30, 30, '{:s} {:s} {}'.format(self.report_title, self.organization, self.test_time))
        if self.current_page_num:
            canvas.drawString(PAGE_WIDTH - 50, 30, '{:d}/{:d}'.format(self.current_page_num, self.page_count))
        self.current_page_num += 1

    def addWatermark(self, canvas, report):
        canvas.saveState()
        canvas.setFont('Helvetica', 50)
        canvas.setFillGray(0.7, 0.5)
        canvas.rotate(45)
        if self.current_page_num % 2 == 0:
            canvas.drawCentredString(300, 400, "Confidential")
        else:
            canvas.drawCentredString(300, 400, "Non-Confidential")
        canvas.restoreState()

    def addTitlePage(self, title, page_type, stamp=None, sub_title_list=None, sub_color_list=None):
        if sub_color_list is None:
            sub_color_list = []
        if sub_title_list is None:
            sub_title_list = []
        self.content.append(PageBreak())
        if page_type == 'sequence':
            self.content.append(Spacer(1, 180))
            self.content.append(Paragraph(title, self.sequence_title_style))
        else:
            self.content.append(Spacer(1, 180))
            self.content.append(Paragraph(title, self.case_title_style))
        self.content.append(Spacer(1, 50))

        if stamp is not None:
            self.content.append(Paragraph(stamp, self.stamp_style))
            self.content.append(Spacer(1, 20))

        for i, sub_title in enumerate(sub_title_list):
            if i not in sub_color_list:
                self.content.append(Spacer(1, 15))
                self.content.append(Paragraph(sub_title, self.sub_title_style))
            else:
                self.content.append(Spacer(1, 15))
                self.content.append(Paragraph(sub_title, self.sub_title_style_1))
        self.page_count += 1

    def addTablePage(self, heading, df_data, span_list, page_count=1):
        self.content.append(PageBreak())
        self.content.append(Paragraph(heading, self.heading_style))
        self.content.append(Spacer(1, 30))

        df_list = [list(df_data.columns)]
        for idx, row in df_data.iterrows():
            df_list.append(list(row.values))

        table = Table(df_list)
        style_list = [
            ("FONT", (0, 0), (-1, 0), 'YaHeiB', 8),
            ('TEXTCOLOR', (0, 0), (-1, 0), colors.steelblue),
            ("FONT", (0, 1), (0, -1), 'YaHeiB', 6),
            ('TEXTCOLOR', (0, 1), (0, -1), colors.steelblue),
            ("FONT", (1, 1), (-1, -1), 'YaHei', 8),
            ('TEXTCOLOR', (1, 1), (-1, -1), colors.black),
            ('ALIGN', (0, 0), (-1, -1), 'CENTER'),
            ('VALIGN', (0, 0), (-1, -1), 'MIDDLE'),
            ('BACKGROUND', (0, 0), (-1, -1), colors.linen),
            ('BOX', (-1, 0), (-1, -1), 3, colors.white),
            ('BOX', (0, 0), (-1, 0), 3, colors.white),
            ('BOX', (0, 0), (0, -1), 3, colors.white),
        ]

        for span in span_list:
            style_list.append(
                ('SPAN', (0, span[0] + 1), (0, span[1] + 1))
            )
            style_list.append(
                ('BOX', (0, span[0] + 1), (-1, span[1] + 1), 1, colors.white),
            )

        table.setStyle(TableStyle(style_list))

        self.content.append(table)
        self.page_count += page_count

    def addOnePage(self, heading, text_list=None, text_spacer=10, df_data=None, img_list=None, img_spacer=5,
                   height_ratio=None):
        if df_data is None:
            df_data = []
        if height_ratio is None:
            height_ratio = []
        if img_list is None:
            img_list = []
        if text_list is None:
            text_list = []

        self.content.append(PageBreak())
        self.content.append(Paragraph(heading, self.heading_style))
        self.content.append(Spacer(1, 10))
        available_height = PAGE_HEIGHT - self.topMargin - self.bottomMargin - 20

        for text in text_list:
            self.content.append(Spacer(1, text_spacer))
            self.content.append(Paragraph(text, self.content_style))
        available_height = available_height - len(text_list) * 22 - 10

        if len(df_data) > 0:
            self.content.append(Spacer(1, text_spacer))
            df_list = [list(df_data.columns)]
            for idx, row in df_data.iterrows():
                df_list.append(list(row.values))

            table = Table(df_list)
            style_list = [
                ("FONT", (0, 0), (-1, 0), 'YaHeiB', 7),
                ('TEXTCOLOR', (0, 0), (-1, 0), colors.steelblue),
                ("FONT", (0, 1), (0, -1), 'YaHeiB', 7),
                ('TEXTCOLOR', (0, 1), (0, -1), colors.steelblue),
                ("FONT", (1, 1), (-1, -1), 'YaHei', 7),
                ('TEXTCOLOR', (1, 1), (-1, -1), colors.black),
                ('ALIGN', (0, 0), (-1, -1), 'CENTER'),
                ('VALIGN', (0, 0), (-1, -1), 'MIDDLE'),
                ('BACKGROUND', (0, 0), (-1, -1), colors.linen),
                ('BOX', (-1, 0), (-1, -1), 3, colors.white),
                ('BOX', (0, 0), (-2, 0), 3, colors.white),
                ('BOX', (0, 0), (0, -1), 3, colors.white),
            ]
            table.setStyle(TableStyle(style_list))
            self.content.append(table)
            available_height = available_height - (len(df_data) + 1) * 17 - 10

        if not len(img_list):
            return
        img_ratio = 1.02
        available_height = (available_height - 5) / img_ratio
        available_width = PAGE_WIDTH - self.leftMargin - self.rightMargin
        for i, img_row in enumerate(img_list):
            if not len(height_ratio):
                img_row_height = available_height / len(img_list)
            else:
                img_row_height = available_height * height_ratio[i] / sum(height_ratio)
            self.content.append(Spacer(1, img_spacer))
            img_group = Drawing()
            img_width_space = available_width / len(img_row) - img_spacer
            img_height_space = img_row_height - img_spacer
            img_group.height = img_height_space
            img_group.width = available_width
            for j, one_img in enumerate(img_row):
                width, height = Image.open(one_img).size
                ratio_w = img_width_space / width
                ratio_h = img_height_space / height
                ratio = min(ratio_w, ratio_h)
                width_r = ratio * width
                height_r = ratio * height
                height_offset = (img_height_space - height_r) / 2
                width_offset = (img_width_space - width_r) / 2
                img_group.add(DrawingImage(j * (img_width_space + img_spacer) + width_offset + img_spacer,
                                           height_offset,
                                           width_r,
                                           height_r, one_img))
            self.content.append(img_group)
        self.page_count += 1

    def genReport(self, folder='', compress=0, report_path=None):
        if report_path:
            path = report_path
        else:
            path = os.path.join(folder, self.base_report_path)

        if compress:
            temp_path = os.path.join(self.project_path, 'temp.pdf')
            temp_path2 = os.path.join(self.project_path, 'Temp', 'temp2.pdf')
            report = SimpleDocTemplate(temp_path, pagesize=landscape(A4),
                                       rightMargin=self.rightMargin, leftMargin=self.leftMargin,
                                       topMargin=self.topMargin, bottomMargin=self.bottomMargin,
                                       )
            # frame = Frame(report.leftMargin, report.bottomMargin, report.width, report.height)
            # template = PageTemplate(id='watermark', frames=[frame], onPage=self.addWatermark)
            # report.addPageTemplates([template])
            report.build(self.content, onFirstPage=self.FirstPage, onLaterPages=self.LaterPages)

            cmd = 'cd {:s}; python3 Api_CompressPdf.py {:s} -o {:s}'.format(
                os.path.join(self.project_path, 'Envs', 'Master', 'Interfaces'), temp_path, temp_path2)
            p = os.popen(cmd)
            p.read()
            shutil.copyfile(temp_path2, path)
            os.remove(temp_path)
            os.remove(temp_path2)
        else:
            report = SimpleDocTemplate(
                path,
                pagesize=landscape(A4), rightMargin=self.rightMargin,
                leftMargin=self.leftMargin, topMargin=self.topMargin, bottomMargin=self.bottomMargin,
            )
            # frame = Frame(report.leftMargin, report.bottomMargin, report.width, report.height)
            # template = PageTemplate(id='watermark', frames=[frame], onPage=self.addWatermark)
            # report.addPageTemplates([template])
            report.build(self.content, onFirstPage=self.FirstPage, onLaterPages=self.LaterPages)

        return path

    def reset(self):
        self.page_count = 0
        self.content = []
        self.current_page_num = 0
