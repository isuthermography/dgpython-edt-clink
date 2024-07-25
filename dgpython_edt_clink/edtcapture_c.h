
static inline void convert_edt_image(unsigned char* gotbuf,unsigned char* recdata, unsigned bytes_per_pixel, unsigned ImgWidth, unsigned ImgHeight, int discardtopline,int FlipLR, int FlipUD)
{
  unsigned char* gotbufstart = gotbuf;
  if(discardtopline) {
    gotbufstart+=ImgWidth*bytes_per_pixel;
  }
  if( !FlipLR && !FlipUD) {
    //simple copy
    memcpy(recdata,gotbufstart,ImgWidth*ImgHeight*bytes_per_pixel);
  } else {
    if(bytes_per_pixel==1) {
      unsigned row,col,destrow,destrowinc,destcol, destcolinc;
      if(FlipLR) {
	destcolinc=-1;
      } else {
	destcolinc=1;
      }
      if(FlipUD){
	destrowinc=-1;
	destrow=ImgHeight-1;
      }   else {
	destrowinc=1;
	destrow=0;
      }
      for (row=0;row<ImgHeight;row++,destrow+=destrowinc) {
	if(FlipLR) {
	  destcol=ImgWidth-1;
	} else {
	  destcol=0;
	}
	for (col=0;col<ImgWidth;col++,destcol+=destcolinc) {
	  recdata[destrow*ImgWidth + destcol] = gotbufstart[row*ImgWidth + col];
	}
      }
    } else {
      uint16_t *recdata16=(uint16_t*)recdata;
      uint16_t *gotbufstart16=(uint16_t*)gotbufstart;
      unsigned row,col,destrow,destrowinc,destcol, destcolinc;
      assert(bytes_per_pixel==2);
      if(FlipLR) {
	destcolinc=-1;
      } else {
	destcolinc=1;
      }
      if(FlipUD){
	destrowinc=-1;
	destrow=ImgHeight-1;
      }   else {
	destrowinc=1;
	destrow=0;
      }
      for (row=0;row<ImgHeight;row++,destrow+=destrowinc) {
	if(FlipLR) {
	  destcol=ImgWidth-1;
	} else {
	  destcol=0;
	}
	for (col=0;col<ImgWidth;col++,destcol+=destcolinc) {
	  recdata16[destrow*ImgWidth + destcol] = gotbufstart16[row*ImgWidth + col];
	}
      }
    }
    
  }
  
  
}

