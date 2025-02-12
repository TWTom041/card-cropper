// A helper to solve an 8x8 linear system by Gaussian elimination.
function solveLinearSystem(A, b) {
	var n = A.length;
	// Augment A with b.
	for (var i = 0; i < n; i++) {
		A[i].push(b[i]);
	}
	// Gaussian elimination.
	for (var i = 0; i < n; i++) {
		// Find pivot.
		var maxRow = i;
		for (var k = i + 1; k < n; k++) {
			if (Math.abs(A[k][i]) > Math.abs(A[maxRow][i])) {
				maxRow = k;
			}
		}
		// Swap rows.
		var tmp = A[i];
		A[i] = A[maxRow];
		A[maxRow] = tmp;
		// Check for near–zero pivot.
		var pivot = A[i][i];
		if (Math.abs(pivot) < 1e-10) continue;
		// Normalize the row.
		for (var j = i; j < n + 1; j++) {
			A[i][j] /= pivot;
		}
		// Eliminate column i in the other rows.
		for (var k = 0; k < n; k++) {
			if (k === i) continue;
			var factor = A[k][i];
			for (var j = i; j < n + 1; j++) {
				A[k][j] -= factor * A[i][j];
			}
		}
	}
	// Extract solution.
	var x = [];
	for (var i = 0; i < n; i++) {
		x.push(A[i][n]);
	}
	return x;
}

// Compute the homography matrix H (3×3) that maps points in destination (x,y)
// to corresponding points in source (u,v). We assume:
//   [u v 1]^T ~ H * [x y 1]^T
// and set H[2][2] = 1.
// The unknowns are h11, h12, h13, h21, h22, h23, h31, h32.
function computeHomography(srcPts, dstPts) {
	// For each correspondence between a destination point (x,y)
	// and source point (u,v), add two equations:
	//   h11*x + h12*y + h13 - u*(h31*x + h32*y + 1) = 0
	//   h21*x + h22*y + h23 - v*(h31*x + h32*y + 1) = 0
	// We rearrange these into the form:
	//   [ x, y, 1, 0, 0, 0, -u*x, -u*y ] * h = u
	//   [ 0, 0, 0, x, y, 1, -v*x, -v*y ] * h = v
	var A = [];
	var b = [];
	for (var i = 0; i < 4; i++) {
		var x = dstPts[i][0], y = dstPts[i][1];
		var u = srcPts[i][0], v = srcPts[i][1];
		A.push([x, y, 1, 0, 0, 0, -u * x, -u * y]);
		b.push(u);
		A.push([0, 0, 0, x, y, 1, -v * x, -v * y]);
		b.push(v);
	}
	var h = solveLinearSystem(A, b);
	// Append the fixed element h33 = 1.
	h.push(1);
	// Return H as a 3×3 matrix.
	return [
		[h[0], h[1], h[2]],
		[h[3], h[4], h[5]],
		[h[6], h[7], h[8]]
	];
}

// Given an image, source quadrilateral, and output size, apply perspective correction.
function applyPerspectiveCorrection(img, srcPts, outputWidth, outputHeight) {
	// Define destination (target) points: the corners of the output rectangle.
	var dstPts = [
		[0, 0],
		[outputWidth, 0],
		[outputWidth, outputHeight],
		[0, outputHeight]
	];
	// Compute the homography H mapping destination -> source.
	// (We use destination points as (x,y) and source points as (u,v) above.)
	var H = computeHomography(srcPts, dstPts);

	// Create a canvas to hold the source image.
	var srcCanvas = document.createElement('canvas');
	srcCanvas.width = img.width;
	srcCanvas.height = img.height;
	var srcCtx = srcCanvas.getContext('2d');
	srcCtx.drawImage(img, 0, 0);
	var srcData = srcCtx.getImageData(0, 0, img.width, img.height);

	// Create an output canvas.
	var dstCanvas = document.createElement('canvas');
	dstCanvas.width = outputWidth;
	dstCanvas.height = outputHeight;
	var dstCtx = dstCanvas.getContext('2d');
	var dstImageData = dstCtx.createImageData(outputWidth, outputHeight);

	// For every pixel in the output image, compute its corresponding source coordinate.
	// The mapping is:
	//    [u v 1]^T ~ H * [x y 1]^T
	for (var y = 0; y < outputHeight; y++) {
		for (var x = 0; x < outputWidth; x++) {
			// Compute the homogeneous coordinate.
			var w = H[2][0] * x + H[2][1] * y + H[2][2];
			var u = (H[0][0] * x + H[0][1] * y + H[0][2]) / w;
			var v = (H[1][0] * x + H[1][1] * y + H[1][2]) / w;

			// Use nearest-neighbor sampling.
			var srcX = Math.round(u);
			var srcY = Math.round(v);
			if (srcX >= 0 && srcX < img.width && srcY >= 0 && srcY < img.height) {
				var srcIndex = (srcY * img.width + srcX) * 4;
				var dstIndex = (y * outputWidth + x) * 4;
				dstImageData.data[dstIndex] = srcData.data[srcIndex];
				dstImageData.data[dstIndex + 1] = srcData.data[srcIndex + 1];
				dstImageData.data[dstIndex + 2] = srcData.data[srcIndex + 2];
				dstImageData.data[dstIndex + 3] = srcData.data[srcIndex + 3];
			}
		}
	}
	dstCtx.putImageData(dstImageData, 0, 0);
	return dstCanvas;
}
