#include "jpeg81.h"
#include "idct.h"
#include "GETC.h"
#include "rb_glue.h"

#include "../imageviewer.h"


/**************** begin Application ********************/

/************************* Types ***************************/

struct t_disp
{
    unsigned char* bitmap;
};

/************************* Globals ***************************/

/* decompressed image in the possible sizes (1,2,4,8), wasting the other */
static struct t_disp disp[9]; /* up to 9 displays (for out of memory case) */

static struct JPEGD jpg; /* too large for stack */

// Streaming mode data
static struct image_info *streaming_info = NULL;
static int streaming_ds = 0;
static fb_data *streaming_bitmap = NULL;

/************************* Implementation ***************************/

static void draw_image_rect(struct image_info *info,
                            int x, int y, int width, int height)
{
    struct t_disp* pdisp = (struct t_disp*)info->data;
#ifdef HAVE_LCD_COLOR
    rb->lcd_bitmap_part(
        (fb_data*)pdisp->bitmap, info->x + x, info->y + y,
        STRIDE(SCREEN_MAIN, info->width, info->height),
        x + MAX(0, (LCD_WIDTH-info->width)/2),
        y + MAX(0, (LCD_HEIGHT-info->height)/2),
        width, height);
#else
    mylcd_ub_gray_bitmap_part(
            pdisp->bitmap, info->x + x, info->y + y, info->width,
            x + MAX(0, (LCD_WIDTH-info->width)/2),
            y + MAX(0, (LCD_HEIGHT-info->height)/2),
            width, height);
#endif
}

static int img_mem(int ds)
{
    struct JPEGD* j = &jpg;
    return j->Y/ds * j->X/ds*sizeof(fb_data);
}

/* my memory pool (from the mp3 buffer) */
static char print[32]; /* use a common snprintf() buffer */

// Streaming callback function
static int jpeg_streaming_callback(struct JPEGD *j, struct JPEG_STREAM_CHUNK *chunk, void *callback_data)
{
    (void)callback_data; // Unused parameter
    
    if (!streaming_info || !streaming_bitmap) {
        return -1; // Error: streaming not properly initialized
    }
    
    // Convert YUV chunk to RGB and store in bitmap
    int h0 = j->Hmax / j->Components[0].Hi;
    int v0 = j->Vmax / j->Components[0].Vi;
    int h1 = j->Hmax / j->Components[1].Hi;
    int v1 = j->Vmax / j->Components[1].Vi;
    int h2 = j->Hmax / j->Components[2].Hi;
    int v2 = j->Vmax / j->Components[2].Vi;
    
    int start_y = chunk->start_y;
    int end_y = start_y + chunk->chunk_height;
    
    for (int y = start_y; y < end_y && y < j->Y; y++) {
        if (y % streaming_ds != 0)
            continue;
            
        // Calculate base array indices with bounds checking
        int y0_base = j->Components[0].du_width * ((y / v0) / 8);
        int y1_base = j->Components[1].du_width * ((y / v1) / 8);
        int y2_base = j->Components[2].du_width * ((y / v2) / 8);
        
        // Bounds checking to prevent array access violations
        if (y0_base >= j->Components[0].du_total ||
            y1_base >= j->Components[1].du_total ||
            y2_base >= j->Components[2].du_total) {
            continue; // Skip this row if any component is out of bounds
        }
        
        // Calculate row offsets within data units with bounds checking
        int y0_offset = ((y / v0) & 7);
        int y1_offset = ((y / v1) & 7);
        int y2_offset = ((y / v2) & 7);
        
        // Ensure row offsets don't exceed data unit bounds (each DU has 8x8 = 64 coefficients)
        if (y0_offset >= 8 || y1_offset >= 8 || y2_offset >= 8) {
            continue; // Skip this row if any offset is out of bounds
        }
        
        fb_data *row_ptr = streaming_bitmap + (y / streaming_ds) * (j->X / streaming_ds);
        
        for (int x = 0; x < j->X; x++) {
            if (x % streaming_ds != 0)
                continue;
                
            // Calculate coefficient indices within the current data unit (0-63)
            // Each data unit is 8x8 coefficients, so we only need the offset within the DU
            int x0_idx = ((x / h0) & 7);
            int x1_idx = ((x / h1) & 7);
            int x2_idx = ((x / h2) & 7);
            
            // Calculate which data unit we're accessing for x coordinate
            int du_x0 = (x / h0) / 8;
            int du_x1 = (x / h1) / 8;
            int du_x2 = (x / h2) / 8;
            
            // Bounds checking: ensure we don't go beyond the row of data units
            if (du_x0 >= j->Components[0].du_w ||
                du_x1 >= j->Components[1].du_w ||
                du_x2 >= j->Components[2].du_w) {
                continue; // Skip this pixel if any component DU is out of bounds
            }
            
            // Ensure coefficient indices are within data unit bounds (0-7 for each dimension)
            if (x0_idx >= 8 || x1_idx >= 8 || x2_idx >= 8) {
                continue; // Skip this pixel if any coefficient index is out of bounds
            }
            
            // Access coefficients correctly:
            // Each data unit is 8x8, so we need the data unit at column du_x*
            // then the row y*_offset within that DU, then the column x*_idx within that row
            TCOEF *du0 = j->Components[0].du[y0_base + du_x0];
            TCOEF *du1 = j->Components[1].du[y1_base + du_x1]; 
            TCOEF *du2 = j->Components[2].du[y2_base + du_x2];
            
            // Additional bounds check for the data unit access
            if (y0_base + du_x0 >= j->Components[0].du_total ||
                y1_base + du_x1 >= j->Components[1].du_total ||
                y2_base + du_x2 >= j->Components[2].du_total) {
                continue; // Skip this pixel if any DU access is out of bounds
            }
            
            TCOEF c0 = du0[y0_offset * 8 + x0_idx];
            TCOEF c1 = du1[y1_offset * 8 + x1_idx];
            TCOEF c2 = du2[y2_offset * 8 + x2_idx];
            
            // ITU BT.601 full-range YUV-to-RGB integer approximation
            int y_val = (c0 << 5) + 16;
            int u = c1 - 128;
            int v = c2 - 128;
            
            // Calculate color component indices with bounds checking
            // CLIP array valid range: -256 to 511
            int b_idx = (y_val + 57 * u) >> 5;
            int g_idx = (y_val - 11 * u - 23 * v) >> 5;
            int r_idx = (y_val + 45 * v) >> 5;
            
            // Clamp indices to valid CLIP array range
            if (b_idx < -256) b_idx = -256;
            else if (b_idx > 511) b_idx = 511;
            if (g_idx < -256) g_idx = -256;
            else if (g_idx > 511) g_idx = 511;
            if (r_idx < -256) r_idx = -256;
            else if (r_idx > 511) r_idx = 511;
            
            int b = CLIP[b_idx];
            int g = CLIP[g_idx];
            int r = CLIP[r_idx];
            
            row_ptr[x / streaming_ds] = FB_RGBPACK(r, g, b);
        }
    }
    
    return 0; // Success
}

static void scaled_dequantization_and_idct(void)
{
    struct JPEGD* j = &jpg;
    // The following code is based on RAINBOW lib jpeg2bmp example:
    // https://github.com/Halicery/vc_rainbow/blob/605c045a564dad8e2df84e48914eac3d2d8d4a9b/jpeg2bmp.c

    printf("Scaled de-quantization and IDCT.. ");
    int c, i, n;

    // Pre-scale quant-tables
    int SQ[4][64];
    for (c=0; c<4 && j->QT[c][0]; c++)
    {
        int *q= j->QT[c], *sq= SQ[c];
        for (i=0; i<64; i++) sq[i]= q[i] * SCALEM[zigzag[i]];
    }

    // DEQUANT + IDCT
    for (c=0; c<j->Nf; c++)
    {
        struct COMP *C= j->Components+c;
        //int *q= j->QT[C->Qi];
        int *sq= SQ[C->Qi];

        for (n=0; n < C->du_size; n++)
        {
            /*
            // <--- scaled idct
            int k, t[64];
            TCOEF *coef= du[x];
            t[0]= coef[0] * q[0] + 1024;							// dequant DC and level-shift (8-bit)
            for (k=1; k<64; k++) t[zigzag[k]] = coef[k] * q[k];		// dequant AC (+zigzag)
            idct_s(t, coef);
            */

            // <--- scaled idct with dequant
            idct_sq( C->du[ (n / C->du_w) * C->du_width +  n % C->du_w ], sq );
        }
    }
    printf("done\n");
}

static int load_image(char *filename, struct image_info *info,
    unsigned char *buf, ssize_t *buf_size,
    int offset, int filesize, int flags)
{
    (void)filesize;(void)flags;
    int status;
    struct JPEGD *p_jpg = &jpg;

    memset(&disp, 0, sizeof(disp));
    memset(&jpg, 0, sizeof(jpg));

    init_mem_pool(buf, *buf_size);

    if (!OPEN(filename))
    {
        return PLUGIN_ERROR;
    }
    if (offset)
    {
        POS(offset);
    }

    if (!iv->running_slideshow)
    {
        rb->lcd_puts(0, 0, rb->strrchr(filename,'/')+1);
        rb->lcd_puts(0, 2, "decoding...");
        rb->lcd_update();
    }
    long time; /* measured ticks */

    /* the actual decoding */
    time = *rb->current_tick;
    
    // Initialize stream state
    memset(&p_jpg->stream_state, 0, sizeof(p_jpg->stream_state));
    
    // Step 1: Parse headers only to determine dimensions and streaming needs
    status = JPEGParseHeaders(p_jpg);
    if (status != JPEGENUM_HEADERS_PARSED) {
        // Header parsing failed, abort
        time = *rb->current_tick - time;
        CLOSE();
        if (status == JPEGENUMERR_MALLOC)
        {
            return PLUGIN_OUTOFMEM;
        }
        rb->splashf(HZ, "header parse error %d", status);
        return PLUGIN_ERROR;
    }
    
    // Step 2: Set up image dimensions and prepare for possible streaming mode
    info->x_size = p_jpg->X;
    info->y_size = p_jpg->Y;
    
    // Pre-initialize streaming globals in case we need them
    streaming_info = info;
    streaming_ds = 1; // Default downscaling
    
    // Step 3: Check if we should use streaming mode and initialize if needed
    if (jpeg_should_use_streaming(p_jpg)) {
        if (!iv->running_slideshow)
        {
            rb->lcd_puts(0, 3, "using streaming mode...");
            rb->lcd_update();
        }
        
        // Try progressive downscaling to find a suitable memory size
        // Supported downscale values: 8, 4, 2, 1
        int downscale_values[] = {1, 2, 4, 8};
        fb_data *temp_bitmap = NULL;
        int selected_ds = 1;
        
        for (int i = 0; i < 4; i++) {
            int ds = downscale_values[i];
            int bitmap_size = img_mem(ds);
            temp_bitmap = (fb_data *)malloc(bitmap_size);
            
            if (temp_bitmap) {
                selected_ds = ds;
                if (!iv->running_slideshow) {
                    rb->lcd_putsf(0, 4, "downscale: %dx", ds);
                    rb->lcd_update();
                }
                break;
            }
        }
        
        if (!temp_bitmap) {
            if (!iv->running_slideshow) {
                rb->lcd_puts(0, 4, "insufficient memory for streaming");
                rb->lcd_update();
            }
            printf("Failed to allocate streaming bitmap buffer, falling back to traditional\n");
        } else {
            streaming_bitmap = temp_bitmap;
            streaming_ds = selected_ds;
            
            // Update info dimensions for downscaled image
            info->x_size = p_jpg->X / selected_ds;
            info->y_size = p_jpg->Y / selected_ds;
            
            // Initialize streaming with our callback
            if (!jpeg_init_streaming(p_jpg, jpeg_streaming_callback, info)) {
                printf("Failed to initialize streaming mode, falling back to traditional\n");
                free(temp_bitmap);
                streaming_bitmap = NULL;
                // Restore original dimensions
                info->x_size = p_jpg->X;
                info->y_size = p_jpg->Y;
            }
        }
    }
    
    // Step 4: Perform the actual image decoding
    status = JPEGDecodeImage(p_jpg);
    
    time = *rb->current_tick - time;

    CLOSE();

    if (status < 0)
    {   /* bad format or minimum components not contained */
        if (status == JPEGENUMERR_MALLOC)
        {
            return PLUGIN_OUTOFMEM;
        }
        rb->splashf(HZ, "unsupported %d", status);
        return  PLUGIN_ERROR;
    }

    if (!iv->running_slideshow)
    {
        rb->lcd_putsf(0, 2, "image %dx%d", info->x_size, info->y_size);
        int w, h; /* used to center output */
        rb->snprintf(print, sizeof(print), "jpegp %ld.%02ld sec ", time/HZ, time%HZ);
        rb->lcd_getstringsize(print, &w, &h); /* centered in progress bar */
        rb->lcd_putsxy((LCD_WIDTH - w)/2, LCD_HEIGHT - h, print);
        rb->lcd_update();
        //rb->sleep(100);
    }

#ifdef DISK_SPINDOWN
    if (iv->running_slideshow && iv->immediate_ata_off)
    {
        /* running slideshow and time is long enough: power down disk */
        rb->storage_sleep();
    }
#endif

    if ( 3 != p_jpg->Nf )
        return PLUGIN_ERROR;

    scaled_dequantization_and_idct();

    *buf_size = freeze_mem_pool();
    return PLUGIN_OK;
}

static int get_image(struct image_info *info, int frame, int ds)
{
    (void)frame;
    struct JPEGD* p_jpg = &jpg;
    struct t_disp* p_disp = &disp[ds]; /* short cut */

    info->width = p_jpg->X / ds;
    info->height = p_jpg->Y / ds;
    info->data = p_disp;

    if (p_disp->bitmap != NULL)
    {
        /* we still have it */
        return PLUGIN_OK;
    }

    struct JPEGD* j = p_jpg;
    int mem = img_mem(ds);

    p_disp->bitmap = malloc(mem);

    if (!p_disp->bitmap)
    {
        clear_mem_pool();
        memset(&disp, 0, sizeof(disp));
        p_disp->bitmap = malloc(mem);
        if (!p_disp->bitmap)
            return PLUGIN_ERROR;
    }

    fb_data *bmp = (fb_data *)p_disp->bitmap;

    // Check if this image was decoded using streaming mode
    if (streaming_bitmap != NULL) {
        printf("Using pre-decoded streaming bitmap\n");
        
        // Copy the streaming bitmap to the display bitmap
        size_t bitmap_size = (info->x_size / ds) * (info->y_size / ds) * sizeof(fb_data);
        if (ds == streaming_ds) {
            // Direct copy if downsampling matches
            rb->memcpy(bmp, streaming_bitmap, bitmap_size);
        } else {
            // Need to resample - for now, just use streaming result as-is
            // TODO: Implement proper resampling for different ds values
            printf("Warning: downsampling mismatch between streaming (%d) and requested (%d)\n", 
                   streaming_ds, ds);
            rb->memcpy(bmp, streaming_bitmap, bitmap_size);
        }
        
        return 0;
    }

    // The following code is based on RAINBOW lib jpeg2bmp example:
    // https://github.com/Halicery/vc_rainbow/blob/605c045a564dad8e2df84e48914eac3d2d8d4a9b/jpeg2bmp.c
    // Primitive yuv-rgb converter for all sub-sampling types, 24-bit BMP only
    printf("YUV-to-RGB conversion.. ");
    
    // Set up streaming globals for the callback (in case we need them)
    streaming_info = info;
    streaming_ds = ds;
    streaming_bitmap = bmp;
    
    int h0 = j->Hmax / j->Components[0].Hi;
    int v0 = j->Vmax / j->Components[0].Vi;
    int h1 = j->Hmax / j->Components[1].Hi;
    int v1 = j->Vmax / j->Components[1].Vi;
    int h2 = j->Hmax / j->Components[2].Hi;
    int v2 = j->Vmax / j->Components[2].Vi;

    int x, y;
    for (y = 0; y < j->Y; y++)
    {
        if (y%ds != 0)
            continue;

        TCOEF *C0 =
                j->Components[0].du[j->Components[0].du_width * ((y / v0) / 8)] + 8 * ((y / v0) & 7);
        TCOEF *C1 =
                j->Components[1].du[j->Components[1].du_width * ((y / v1) / 8)] + 8 * ((y / v1) & 7);
        TCOEF *C2 =
                j->Components[2].du[j->Components[2].du_width * ((y / v2) / 8)] + 8 * ((y / v2) & 7);

        for (x = 0; x < j->X; x++)
        {
            if (x%ds != 0)
                continue;

            TCOEF c0 = C0[(x / h0 / 8) * 64 + ((x / h0) & 7)];
            TCOEF c1 = C1[(x / h1 / 8) * 64 + ((x / h1) & 7)];
            TCOEF c2 = C2[(x / h2 / 8) * 64 + ((x / h2) & 7)];

            // ITU BT.601 full-range YUV-to-RGB integer approximation 
            {
                int y = (c0 << 5) + 16;
                int u = c1 - 128;
                int v = c2 - 128;

                // Calculate color component indices with bounds checking
                // CLIP array valid range: -256 to 511
                int b_idx = (y + 57 * u) >> 5;
                int g_idx = (y - 11 * u - 23 * v) >> 5;
                int r_idx = (y + 45 * v) >> 5;
                
                // Clamp indices to valid CLIP array range
                if (b_idx < -256) b_idx = -256;
                else if (b_idx > 511) b_idx = 511;
                if (g_idx < -256) g_idx = -256;
                else if (g_idx > 511) g_idx = 511;
                if (r_idx < -256) r_idx = -256;
                else if (r_idx > 511) r_idx = 511;
                
                int b = CLIP[b_idx];		// B;
                int g = CLIP[g_idx];	// G
                int r = CLIP[r_idx];		// R;
                *bmp++= FB_RGBPACK(r,g,b);
            }
        }
    }
    printf("done\n");
    return 0;
}

const struct image_decoder image_decoder = {
    false,
    img_mem,
    load_image,
    get_image,
    draw_image_rect,
};

IMGDEC_HEADER
