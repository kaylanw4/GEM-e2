import numpy as np
import cv2
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import pickle

# from combined_thresh import combined_thresh
# from perspective_transform import perspective_transform

# feel free to adjust the parameters in the code if necessary


def line_fit(binary_warped):
    """
    Find and fit lane lines
    """
    # Assuming you have created a warped binary image called "binary_warped"
    # Take a histogram of the bottom half of the image
    histogram = np.sum(binary_warped[binary_warped.shape[0] // 2 :, :], axis=0)
    # breakpoint()
    # Create an output image to draw on and visualize the result
    out_img = (np.dstack((binary_warped, binary_warped, binary_warped)) * 255).astype(
        "uint8"
    )
    # Find the peak of the left and right halves of the histogram
    # These will be the starting point for the left and right lines
    midpoint = np.int(histogram.shape[0] / 2)
    leftx_base = np.argmax(histogram[100:midpoint]) + 100
    rightx_base = np.argmax(histogram[midpoint:-100]) + midpoint

    # Choose the number of sliding windows
    nwindows = 9
    # Set height of windows
    window_height = np.int(binary_warped.shape[0] / nwindows)
    # Identify the x and y positions of all nonzero pixels in the image
    nonzero = binary_warped.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    # Current positions to be updated for each window
    leftx_current = leftx_base
    rightx_current = rightx_base
    # Set the width of the windows +/- margin
    margin = 100
    # Set minimum number of pixels found to recenter window
    minpix = 50
    # Create empty lists to receive left and right lane pixel indices
    left_lane_inds = []
    right_lane_inds = []

    # Step through the windows one by one
    for window in range(nwindows):
        # Identify window boundaries in x and y (and right and left)
        ## TODO
        window_y_low = binary_warped.shape[0] - (window + 1) * window_height
        window_y_high = binary_warped.shape[0] - window * window_height
        window_leftx_low = leftx_current - margin
        window_leftx_high = leftx_current + margin
        window_rightx_low = rightx_current - margin
        window_rightx_high = rightx_current + margin

        ####
        # Draw the windows on the visualization image using cv2.rectangle()
        ## TODO
        cv2.rectangle(
            out_img,
            (window_leftx_low, window_y_low),
            (window_leftx_high, window_y_high),
            (0, 255, 0),
        )
        cv2.rectangle(
            out_img,
            (window_rightx_low, window_y_low),
            (window_rightx_high, window_y_high),
            (0, 255, 0),
        )
        # cv2.imshow('image', out_img)
        # cv2.waitKey(0)
        ####
        # Identify the nonzero pixels in x and y within the window
        ## TODO
        left_inrange_inds = (
            (nonzeroy > window_y_low)
            & (nonzeroy < window_y_high)
            & (nonzerox > window_leftx_low)
            & (nonzerox < window_leftx_high)
        )
        left_inds = np.where(left_inrange_inds)[0]
        right_inrange_inds = (
            (nonzeroy > window_y_low)
            & (nonzeroy < window_y_high)
            & (nonzerox > window_rightx_low)
            & (nonzerox < window_rightx_high)
        )
        right_inds = np.where(right_inrange_inds)[0]

        ####
        # Append these indices to the lists
        ## TODO
        left_lane_inds.append(left_inds)
        right_lane_inds.append(right_inds)

        ####
        # If you found > minpix pixels, recenter next window on their mean position
        ## TODO
        if len(left_inds) > minpix:
            leftx_current = np.int(np.mean(nonzerox[left_inds]))
        if len(right_inds) > minpix:
            rightx_current = np.int(np.mean(nonzerox[right_inds]))

        ####

    # Concatenate the arrays of indices
    left_lane_inds = np.concatenate(left_lane_inds)
    right_lane_inds = np.concatenate(right_lane_inds)

    # Extract left and right line pixel positions
    leftx = nonzerox[left_lane_inds]
    lefty = nonzeroy[left_lane_inds]
    rightx = nonzerox[right_lane_inds]
    righty = nonzeroy[right_lane_inds]

    # Fit a second order polynomial to each using np.polyfit()
    # If there isn't a good fit, meaning any of leftx, lefty, rightx, and righty are empty,
    # the second order polynomial is unable to be sovled.
    # Thus, it is unable to detect edges.
    try:
        ## TODO
        left_fit = np.polyfit(lefty, leftx, 2)
        right_fit = np.polyfit(righty, rightx, 2)
    ####
    except TypeError:
        print("Unable to detect lanes")
        return None

    # Return a dict of relevant variables
    ret = {}
    ret["left_fit"] = left_fit
    ret["right_fit"] = right_fit
    ret["nonzerox"] = nonzerox
    ret["nonzeroy"] = nonzeroy
    ret["out_img"] = out_img
    ret["left_lane_inds"] = left_lane_inds
    ret["right_lane_inds"] = right_lane_inds

    return ret


def line_fit_left(binary_warped):
    """
    Find and fit lane lines
    """
    # Assuming you have created a warped binary image called "binary_warped"
    # Take a histogram of the bottom half of the image
    histogram = np.sum(binary_warped[binary_warped.shape[0] // 2 :, :], axis=0)
    # breakpoint()
    # Create an output image to draw on and visualize the result
    out_img = (np.dstack((binary_warped, binary_warped, binary_warped)) * 255).astype(
        "uint8"
    )
    # Find the peak of the left and right halves of the histogram
    # These will be the starting point for the left and right lines
    midpoint = np.int(histogram.shape[0] / 2)
    leftx_base = max(np.argmax(histogram[0:midpoint]), 100)
    # leftx_base = np.max(np.argmax(histogram[0:midpoint]), 100)

    # Choose the number of sliding windows
    nwindows = 9
    # Set height of windows
    window_height = np.int(binary_warped.shape[0] / nwindows)
    # Identify the x and y positions of all nonzero pixels in the image
    nonzero = binary_warped.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    # Current positions to be updated for each window
    leftx_current = leftx_base
    # Set the width of the windows +/- margin
    margin = 100
    # Set minimum number of pixels found to recenter window
    minpix = 50
    # Create empty lists to receive left and right lane pixel indices
    left_lane_inds = np.zeros_like(nonzerox, dtype=bool)

    # Step through the windows one by one
    for window in range(nwindows):
        # Identify window boundaries in x and y (and right and left)
        ## TODO
        window_y_low = binary_warped.shape[0] - (window + 1) * window_height
        window_y_high = binary_warped.shape[0] - window * window_height
        window_leftx_low = leftx_current - margin
        window_leftx_high = leftx_current + margin

        ####
        # Draw the windows on the visualization image using cv2.rectangle()
        ## TODO
        cv2.rectangle(
            out_img,
            (window_leftx_low, window_y_low),
            (window_leftx_high, window_y_high),
            (0, 255, 0),
        )
        # cv2.imshow('image', out_img)
        # cv2.waitKey(0)
        ####
        # Identify the nonzero pixels in x and y within the window
        ## TODO
        left_inrange_inds = (
            (nonzeroy > window_y_low)
            & (nonzeroy < window_y_high)
            & (nonzerox > window_leftx_low)
            & (nonzerox < window_leftx_high)
        )
        left_inds = np.where(left_inrange_inds)[0]
        ####
        # Append these indices to the lists
        ## TODO
        left_lane_inds = np.logical_or(left_lane_inds, left_inrange_inds)

        ####
        # If you found > minpix pixels, recenter next window on their mean position
        ## TODO
        if len(left_inds) > minpix:
            leftx_current = np.int(np.mean(nonzerox[left_inds]))

        ####

    # print(left_lane_inds)
    # Concatenate the arrays of indices
    # left_lane_inds = np.concatenate(left_lane_inds)

    # Extract left and right line pixel positions
    leftx = nonzerox[left_lane_inds]
    lefty = nonzeroy[left_lane_inds]

    # Fit a second order polynomial to each using np.polyfit()
    # If there isn't a good fit, meaning any of leftx, lefty, rightx, and righty are empty,
    # the second order polynomial is unable to be sovled.
    # Thus, it is unable to detect edges.
    try:
        ## TODO
        left_fit = np.polyfit(lefty, leftx, 2)
    ####
    except TypeError:
        print("[Left] Unable to detect lanes")
        return None

    # Return a dict of relevant variables
    ret = {}
    ret["left_fit"] = left_fit
    ret["nonzerox"] = nonzerox
    ret["nonzeroy"] = nonzeroy
    ret["out_img"] = out_img
    ret["left_lane_inds"] = left_lane_inds

    return ret


def line_fit_right(binary_warped):
    """
    Find and fit lane lines
    """
    # Assuming you have created a warped binary image called "binary_warped"
    # Take a histogram of the bottom half of the image
    histogram = np.sum(binary_warped[binary_warped.shape[0] // 2 :, :], axis=0)
    # breakpoint()
    # Create an output image to draw on and visualize the result
    out_img = (np.dstack((binary_warped, binary_warped, binary_warped)) * 255).astype(
        "uint8"
    )
    # Find the peak of the left and right halves of the histogram
    # These will be the starting point for the left and right lines
    midpoint = np.int(histogram.shape[0] / 2)
    rightx_base = min(np.argmax(histogram[midpoint:]) + midpoint, histogram.shape[0] - 100)

    # Choose the number of sliding windows
    nwindows = 9
    # Set height of windows
    window_height = np.int(binary_warped.shape[0] / nwindows)
    # Identify the x and y positions of all nonzero pixels in the image
    nonzero = binary_warped.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    # Current positions to be updated for each window
    rightx_current = rightx_base
    # Set the width of the windows +/- margin
    margin = 100
    # Set minimum number of pixels found to recenter window
    minpix = 50
    # Create empty lists to receive left and right lane pixel indices
    right_lane_inds = np.zeros_like(nonzerox, dtype=bool)

    # Step through the windows one by one
    for window in range(nwindows):
        # Identify window boundaries in x and y (and right and left)
        ## TODO
        window_y_low = binary_warped.shape[0] - (window + 1) * window_height
        window_y_high = binary_warped.shape[0] - window * window_height
        window_rightx_low = rightx_current - margin
        window_rightx_high = rightx_current + margin

        ####
        # Draw the windows on the visualization image using cv2.rectangle()
        ## TODO
        cv2.rectangle(
            out_img,
            (window_rightx_low, window_y_low),
            (window_rightx_high, window_y_high),
            (0, 255, 0),
        )
        # cv2.imshow('image', out_img)
        # cv2.waitKey(0)
        ####
        # Identify the nonzero pixels in x and y within the window
        ## TODO
        right_inrange_inds = (
            (nonzeroy > window_y_low)
            & (nonzeroy < window_y_high)
            & (nonzerox > window_rightx_low)
            & (nonzerox < window_rightx_high)
        )
        right_inds = np.where(right_inrange_inds)[0]

        ####
        # Append these indices to the lists
        ## TODOoy > window_y_low
        right_lane_inds = np.logical_or(right_lane_inds, right_inrange_inds)

        ####
        # If you found > minpix pixels, recenter next window on their mean position
        ## TODO
        if len(right_inds) > minpix:
            rightx_current = np.int(np.mean(nonzerox[right_inds]))

        ####

    # Concatenate the arrays of indices
    # right_lane_inds = np.concatenate(right_lane_inds)

    # Extract left and right line pixel positions
    rightx = nonzerox[right_lane_inds]
    righty = nonzeroy[right_lane_inds]

    # Fit a second order polynomial to each using np.polyfit()
    # If there isn't a good fit, meaning any of leftx, lefty, rightx, and righty are empty,
    # the second order polynomial is unable to be sovled.
    # Thus, it is unable to detect edges.
    try:
        ## TODO
        right_fit = np.polyfit(righty, rightx, 2)
    ####
    except TypeError:
        print("[Right] Unable to detect lanes")
        return None

    # Return a dict of relevant variables
    ret = {}
    ret["right_fit"] = right_fit
    ret["nonzerox"] = nonzerox
    ret["nonzeroy"] = nonzeroy
    ret["out_img"] = out_img
    ret["right_lane_inds"] = right_lane_inds

    return ret


def tune_fit_left(binary_warped, left_fit):
    """
    Given a previously fit line, quickly try to find the line based on previous lines
    """
    # Assume you now have a new warped binary image
    # from the next frame of video (also called "binary_warped")
    # It's now much easier to find line pixels!
    nonzero = binary_warped.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    margin = 100
    left_lane_inds = (
        nonzerox
        > (
            left_fit[0] * (nonzeroy**2)
            + left_fit[1] * nonzeroy
            + left_fit[2]
            - margin
        )
    ) & (
        nonzerox
        < (
            left_fit[0] * (nonzeroy**2)
            + left_fit[1] * nonzeroy
            + left_fit[2]
            + margin
        )
    )

    # Again, extract left and right line pixel positions
    leftx = nonzerox[left_lane_inds]
    lefty = nonzeroy[left_lane_inds]

    # If we don't find enough relevant points, return all None (this means error)
    min_inds = 10
    if lefty.shape[0] < min_inds:
        return None

    # Fit a second order polynomial to each
    left_fit = np.polyfit(lefty, leftx, 2)
    # Generate x and y values for plotting
    ploty = np.linspace(0, binary_warped.shape[0] - 1, binary_warped.shape[0])
    left_fitx = left_fit[0] * ploty**2 + left_fit[1] * ploty + left_fit[2]

    # Return a dict of relevant variables
    ret = {}
    ret["left_fit"] = left_fit
    ret["nonzerox"] = nonzerox
    ret["nonzeroy"] = nonzeroy
    ret["left_lane_inds"] = left_lane_inds

    return ret


def tune_fit_right(binary_warped, right_fit):
    """
    Given a previously fit line, quickly try to find the line based on previous lines
    """
    # Assume you now have a new warped binary image
    # from the next frame of video (also called "binary_warped")
    # It's now much easier to find line pixels!
    nonzero = binary_warped.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    margin = 100
    right_lane_inds = (
        nonzerox
        > (
            right_fit[0] * (nonzeroy**2)
            + right_fit[1] * nonzeroy
            + right_fit[2]
            - margin
        )
    ) & (
        nonzerox
        < (
            right_fit[0] * (nonzeroy**2)
            + right_fit[1] * nonzeroy
            + right_fit[2]
            + margin
        )
    )

    # Again, extract left and right line pixel positions
    rightx = nonzerox[right_lane_inds]
    righty = nonzeroy[right_lane_inds]

    # If we don't find enough relevant points, return all None (this means error)
    min_inds = 10
    if righty.shape[0] < min_inds:
        return None

    # Fit a second order polynomial to each
    right_fit = np.polyfit(righty, rightx, 2)
    # Generate x and y values for plotting
    ploty = np.linspace(0, binary_warped.shape[0] - 1, binary_warped.shape[0])
    right_fitx = right_fit[0] * ploty**2 + right_fit[1] * ploty + right_fit[2]

    # Return a dict of relevant variables
    ret = {}
    ret["right_fit"] = right_fit
    ret["nonzerox"] = nonzerox
    ret["nonzeroy"] = nonzeroy
    ret["right_lane_inds"] = right_lane_inds

    return ret


def viz1(binary_warped, ret, save_file=None):
    """
    Visualize each sliding window location and predicted lane lines, on binary warped image
    save_file is a string representing where to save the image (if None, then just display)
    """
    # Grab variables from ret dictionary
    left_fit = ret["left_fit"]
    right_fit = ret["right_fit"]
    nonzerox = ret["nonzerox"]
    nonzeroy = ret["nonzeroy"]
    out_img = ret["out_img"]
    left_lane_inds = ret["left_lane_inds"]
    right_lane_inds = ret["right_lane_inds"]

    # Generate x and y values for plotting
    ploty = np.linspace(0, binary_warped.shape[0] - 1, binary_warped.shape[0])
    left_fitx = left_fit[0] * ploty**2 + left_fit[1] * ploty + left_fit[2]
    right_fitx = right_fit[0] * ploty**2 + right_fit[1] * ploty + right_fit[2]

    out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 0]
    out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [0, 0, 255]
    plt.imshow(out_img)
    plt.plot(left_fitx, ploty, color="yellow")
    plt.plot(right_fitx, ploty, color="yellow")
    plt.xlim(0, 1280)
    plt.ylim(720, 0)
    if save_file is None:
        plt.show()
    else:
        plt.savefig(save_file)
    plt.gcf().clear()


def bird_fit(binary_warped, ret_l, ret_r, save_file=None):
    """
    Visualize the predicted lane lines with margin, on binary warped image
    save_file is a string representing where to save the image (if None, then just display)
    """

    # Create an image to draw on and an image to show the selection window
    out_img = (np.dstack((binary_warped, binary_warped, binary_warped)) * 255).astype(
        "uint8"
    )
    window_img = np.zeros_like(out_img)
    ploty = np.linspace(0, binary_warped.shape[0] - 1, binary_warped.shape[0])
    margin = 100  # NOTE: Keep this in sync with *_fit()

    # Grab variables from ret dictionary
    if ret_l is not None:
        left_fit = ret_l["left_fit"]
        left_lane_inds = ret_l["left_lane_inds"]
        nonzerox = ret_l["nonzerox"]
        nonzeroy = ret_l["nonzeroy"]

        # Color in left and right line pixels
        out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 0]

        # Generate x and y values for plotting
        left_fitx = left_fit[0] * ploty**2 + left_fit[1] * ploty + left_fit[2]

        # Generate a polygon to illustrate the search window area
        # And recast the x and y points into usable format for cv2.fillPoly()
        left_line_window1 = np.array(
            [np.transpose(np.vstack([left_fitx - margin, ploty]))]
        )
        left_line_window2 = np.array(
            [np.flipud(np.transpose(np.vstack([left_fitx + margin, ploty])))]
        )
        left_line_pts = np.hstack((left_line_window1, left_line_window2))

        # Draw the lane onto the warped blank image
        cv2.fillPoly(window_img, np.int_([left_line_pts]), (0, 255, 0))

    if ret_r is not None:
        right_fit = ret_r["right_fit"]
        right_lane_inds = ret_r["right_lane_inds"]
        nonzerox = ret_r["nonzerox"]
        nonzeroy = ret_r["nonzeroy"]

        # Color in left and right line pixels
        out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [0, 0, 255]

        # Generate x and y values for plotting
        right_fitx = right_fit[0] * ploty**2 + right_fit[1] * ploty + right_fit[2]

        # Generate a polygon to illustrate the search window area
        # And recast the x and y points into usable format for cv2.fillPoly()
        right_line_window1 = np.array(
            [np.transpose(np.vstack([right_fitx - margin, ploty]))]
        )
        right_line_window2 = np.array(
            [np.flipud(np.transpose(np.vstack([right_fitx + margin, ploty])))]
        )
        right_line_pts = np.hstack((right_line_window1, right_line_window2))

        # Draw the lane onto the warped blank image
        cv2.fillPoly(window_img, np.int_([right_line_pts]), (0, 255, 0))

    result = cv2.addWeighted(out_img, 1, window_img, 0.3, 0)
    plt.imshow(result)
    if ret_l is not None:
        plt.plot(left_fitx, ploty, color="yellow")
    if ret_r is not None:
        plt.plot(right_fitx, ploty, color="yellow")
    plt.xlim(0, 1280)
    plt.ylim(720, 0)

    # cv2.imshow('bird',result)
    # cv2.imwrite('bird_from_cv2.png', result)

    # if save_file is None:
    # 	plt.show()
    # else:
    # 	plt.savefig(save_file)
    # plt.gcf().clear()

    return result


def final_viz(undist, left_fit, right_fit, m_inv):
    """
    Final lane line prediction visualized and overlayed on top of original image
    """
    # Generate x and y values for plotting
    ploty = np.linspace(0, undist.shape[0] - 1, undist.shape[0])

    # Create an image to draw the lines on
    # warp_zero = np.zeros_like(warped).astype(np.uint8)
    # color_warp = np.dstack((warp_zero, warp_zero, warp_zero))
    color_warp = np.zeros(
        (720, 1280, 3), dtype="uint8"
    )  # NOTE: Hard-coded image dimensions

    pts_left = np.array([[[0,0],[0,720]]])
    pts_right = np.array([[[1280,0],[1280,720]]])
    if left_fit is not None:
        left_fitx = left_fit[0] * ploty**2 + left_fit[1] * ploty + left_fit[2]

        # Recast the x and y points into usable format for cv2.fillPoly()
        pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
    if right_fit is not None:
        right_fitx = right_fit[0] * ploty**2 + right_fit[1] * ploty + right_fit[2]

        # Recast the x and y points into usable format for cv2.fillPoly()
        pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
        # print(pts_right)
    pts = np.hstack((pts_left, pts_right))

    # Draw the lane onto the warped blank image
    # print(np.int_([pts]))
    cv2.fillPoly(color_warp, np.int_([pts]), (0, 255, 0))
    # cv2.imshow('image', color_warp)
    # cv2.waitKey(0)

    # Warp the blank back to original image space using inverse perspective matrix (Minv)
    newwarp = cv2.warpPerspective(color_warp, m_inv, (undist.shape[1], undist.shape[0]))
    # Combine the result with the original image
    # Convert arrays to 8 bit for later cv to ros image transfer
    undist = np.array(undist, dtype=np.uint8)
    newwarp = np.array(newwarp, dtype=np.uint8)
    result = cv2.addWeighted(undist, 1, newwarp, 0.3, 0)

    return result
