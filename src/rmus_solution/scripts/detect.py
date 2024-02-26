#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import traceback
import cv2
import copy
import rospy
import onnxruntime
import numpy as np

onnx_model_path = os.path.join(os.path.dirname(__file__), "model/CNN_v2.onnx")
# 创建ONNX Runtime会话
session = onnxruntime.InferenceSession(onnx_model_path)

def sort_contour(cnt):

    if not len(cnt) == 4:
        assert False
    new_cnt = cnt.copy()

    cx = (cnt[0, 0, 0] + cnt[1, 0, 0] + cnt[2, 0, 0] + cnt[3, 0, 0]) / 4.0
    cy = (cnt[0, 0, 1] + cnt[1, 0, 1] + cnt[2, 0, 1] + cnt[3, 0, 1]) / 4.0

    x_left_n = 0
    for i in range(4):
        if cnt[i, 0, 0] < cx:
            x_left_n += 1
    if x_left_n != 2:
        return None
    lefts = np.array([c for c in cnt if c[0, 0] < cx])
    rights = np.array([c for c in cnt if c[0, 0] >= cx])
    if lefts[0, 0, 1] < lefts[1, 0, 1]:
        new_cnt[0, 0, 0] = lefts[0, 0, 0]
        new_cnt[0, 0, 1] = lefts[0, 0, 1]
        new_cnt[3, 0, 0] = lefts[1, 0, 0]
        new_cnt[3, 0, 1] = lefts[1, 0, 1]
    else:
        new_cnt[0, 0, 0] = lefts[1, 0, 0]
        new_cnt[0, 0, 1] = lefts[1, 0, 1]
        new_cnt[3, 0, 0] = lefts[0, 0, 0]
        new_cnt[3, 0, 1] = lefts[0, 0, 1]

    if rights[0, 0, 1] < rights[1, 0, 1]:
        new_cnt[1, 0, 0] = rights[0, 0, 0]
        new_cnt[1, 0, 1] = rights[0, 0, 1]
        new_cnt[2, 0, 0] = rights[1, 0, 0]
        new_cnt[2, 0, 1] = rights[1, 0, 1]
    else:
        new_cnt[1, 0, 0] = rights[1, 0, 0]
        new_cnt[1, 0, 1] = rights[1, 0, 1]
        new_cnt[2, 0, 0] = rights[0, 0, 0]
        new_cnt[2, 0, 1] = rights[0, 0, 1]
    return new_cnt


def preprocessing_exchange(frame):
    hsvImg = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    boolImg = (
        np.logical_and(
            np.logical_and(
                np.logical_or(hsvImg[:, :, 0] <= 10, hsvImg[:, :, 0] >= 150),
                hsvImg[:, :, 1] >= 60,
            ),
            hsvImg[:, :, 2] >= 75,
        )
        * 255
    ).astype(np.uint8)
    # boolImg = (np.logical_and(np.logical_and(np.logical_or(hsvImg[:,:,0] <= 10, hsvImg[:,:,0] >= 150), hsvImg[:,:,1] >= 100), hsvImg[:,:,2] >= 100) * 255).astype(np.uint8)
    return boolImg, hsvImg


def preprocessing(frame):
    hsvImg = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    boolImg = (
        np.logical_and(
            np.logical_and(
                np.logical_or(hsvImg[:, :, 0] <= 10, hsvImg[:, :, 0] >= 150),
                hsvImg[:, :, 1] >= 60,
            ),
            hsvImg[:, :, 2] >= 50,
        )
        * 255
    ).astype(np.uint8)
    # boolImg = (np.logical_and(np.logical_and(np.logical_or(hsvImg[:,:,0] <= 10 , hsvImg[:,:,0] >= 150) , hsvImg[:,:,1] >= 100) , hsvImg[:,:,2] >= 50) * 255).astype(np.uint8)
    return boolImg, hsvImg


def square_detection(frame, grayImg, camera_matrix, area_filter_size=30, height_range=(-10000.0, 200000.0), template_ids=range(1, 9)):
    global session
    input_name = 'input'
    all_ID = []
    quads_ID = []
    minareas_list = []

    projection_points = True
    quads = []
    quads_f = []
    contours, hierarchy = cv2.findContours(
        grayImg, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE
    )

    try:
        hierarchy = hierarchy[0]
        father_contours = []
        start_search = 0
        while start_search < len(contours):
            if hierarchy[start_search, -1] == -1:
                break
            start_search += 1
        saved_idx = [start_search]
        next_same_level_idx = hierarchy[start_search, 0]
        while next_same_level_idx != -1:
            saved_idx.append(next_same_level_idx)
            next_same_level_idx = hierarchy[next_same_level_idx, 0]
        for i in saved_idx:
            father_contours.append(contours[i])
        contours = father_contours
    except:
        rospy.loginfo("Nothing detected in hierarchy")
        cv2.imwrite("./debug.png", grayImg)

    if area_filter_size < 200:
        filter_len = 4
        projection_points = False
    elif area_filter_size < 250:
        filter_len = 5
        projection_points = False
    elif area_filter_size < 350:
        filter_len = 10
    else:
        filter_len = 15
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < 100.0:
            continue
        if area < 200:
            approx = cv2.approxPolyDP(contour, 4, True)
        else:
            approx = cv2.approxPolyDP(contour, 8, True)

        if len(approx) == 4:
            approx_sort = sort_contour(approx)
            if approx_sort is not None \
                    and abs(approx_sort[0][0][1] - approx_sort[1][0][1]) < abs(
                approx_sort[0][0][1] - approx_sort[3][0][1]) / 3 \
                    and abs(approx_sort[2][0][1] - approx_sort[3][0][1]) < abs(
                approx_sort[0][0][1] - approx_sort[3][0][1]) / 3:
                quads.append(approx_sort)

    dst_quads = []
    for i in range(len(quads)):
        points_src = np.array(
            [
                [(quads[i][0, 0, 0], quads[i][0, 0, 1])],
                [(quads[i][1, 0, 0], quads[i][1, 0, 1])],
                [(quads[i][2, 0, 0], quads[i][2, 0, 1])],
                [(quads[i][3, 0, 0], quads[i][3, 0, 1])],
            ],
            dtype="float32",
        )

        points_dst = np.array([[0, 0], [49, 0], [49, 49], [0, 49]], dtype="float32")
        out_img = cv2.warpPerspective(
            frame, cv2.getPerspectiveTransform(points_src, points_dst), (50, 50)
        )
        out_img = cv2.cvtColor(out_img, cv2.COLOR_BGR2GRAY)
        out_img = cv2.threshold(out_img, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)[1]
<<<<<<< HEAD
        # black = 0
        # white = 0
        # for i1 in range(50):
        #     for j1 in range(50):
        #         if out_img[i1, j1] == 0:
        #             black += 1
        #         else:
        #             white += 1
        # rate1 = white / (50 * 50)
        # if rate1 > 0.8 or rate1 < 0.2:
        #     continue
=======

>>>>>>> b7bfd294bcc1818ba392b0c3f14488cfefe47597
        input_data = out_img.reshape((1, 1, out_img.shape[0], out_img.shape[1])).astype(np.float32)
        output = session.run(None, {input_name: input_data})
        predictions = output[0]
        max_index = np.argmax(predictions)
        if max_index != 0 and (max_index in template_ids):
            dst_quads.append(quads[i])
            quads_ID.append(max_index)
            quads_f.append(quads[i].astype(float))
            minareas_list.append(cv2.contourArea(quads[i]))
        if max_index != 0:
            all_ID.append(max_index)

    if projection_points:
        rvec_list = []
        tvec_list = []
        quads_prj = []
        area_list = []

        block_size = 0.045
        model_object = np.array(
            [
                (0 - 0.5 * block_size, 0 - 0.5 * block_size, 0.0),
                (block_size - 0.5 * block_size, 0 - 0.5 * block_size, 0.0),
                (block_size - 0.5 * block_size, block_size - 0.5 * block_size, 0.0),
                (0 - 0.5 * block_size, block_size - 0.5 * block_size, 0.0),
            ]
        )
        # camera_matrix = np.array(
        #     [
        #         (617.3054000792732, 0.0, 424.0),
        #         (
        #             0.0,
        #             608.3911743164062,
        #             243.64712524414062,
        #         ),
        #         (0, 0, 1),
        #     ],
        #     dtype="double",
        # )
        dist_coeffs = np.array([[0, 0, 0, 0]], dtype="double")
        for quad in quads_f:
            model_image = np.array(
                [
                    (quad[0, 0, 0], quad[0, 0, 1]),
                    (quad[1, 0, 0], quad[1, 0, 1]),
                    (quad[2, 0, 0], quad[2, 0, 1]),
                    (quad[3, 0, 0], quad[3, 0, 1]),
                ]
            )
            ret, rvec, tvec = cv2.solvePnP(
                model_object, model_image, camera_matrix, dist_coeffs
            )
            projectedPoints, _ = cv2.projectPoints(
                model_object, rvec, tvec, camera_matrix, dist_coeffs
            )

            err = 0
            for t in range(len(projectedPoints)):
                err += np.linalg.norm(projectedPoints[t] - model_image[t])

            area = cv2.contourArea(quad.astype(np.int))
            # if (
            #     err / area < 0.005
            #     and tvec[1] > height_range[0]
            #     and tvec[1] < height_range[1]
            # ):
            quads_prj.append(projectedPoints.astype(int))
            rvec_list.append(rvec)
            tvec_list.append(tvec)
            area_list.append(area)
        return quads_prj, tvec_list, rvec_list, area_list, all_ID, quads_ID, minareas_list
    else:
        return (
            dst_quads,
            [[0, 0, 0] for _ in dst_quads],
            [[0, 0, 0] for _ in dst_quads],
            [cv2.contourArea(quad.astype(np.int)) for quad in dst_quads],
            all_ID,
            quads_ID,
            minareas_list
        )


def marker_detection(
    frame,
    camera_matrix,
    template_ids=range(1, 9),
    area_filter_size=30,
    seg_papram=None,
    verbose=True,
    height_range=(-10000.0, 200000.0),
    exchange_station=False,
):
    all_ID = [] # 检测到的所有ID
    minareas_list = [] # 面积列表
    if exchange_station:
        tframe = copy.deepcopy(frame)
        tframe[int(tframe.shape[0] * 0.32) :, :, :] = 0
        boolImg, _ = preprocessing_exchange(tframe)
    else:
        boolImg, _ = preprocessing(frame)
    quads, tvec_list, rvec_list, area_list, all_ID, quads_ID, minareas_list = square_detection(
        frame, boolImg, camera_matrix, area_filter_size=area_filter_size, height_range=height_range, template_ids=template_ids
    )

    if verbose:
        id = {1: "1", 2: "2", 3: "3", 4: "4", 5: "5", 6: "6", 7: "B", 8: "O", 9: "X", 0: "*"}
        for i in range(len(quads)):
            bbox = cv2.boundingRect(quads[i])
            try:
                cv2.putText(
                    frame,
                    id[quads_ID[i]],
                    (bbox[0], bbox[1]),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 255, 255),
                    2,
                )
            except:
                traceback.print_exc()
        cv2.drawContours(frame, quads, -1, (0, 255, 0), 1)
    ids = [i for i in range(len(quads_ID)) if quads_ID[i] >= 1 and quads_ID[i] <= 9]
    return (
        [quads_ID[_] for _ in ids],
        [quads[_] for _ in ids],
        [area_list[_] for _ in ids],
        [tvec_list[_] for _ in ids],
        [rvec_list[_] for _ in ids],
        all_ID,
        minareas_list,
    )
