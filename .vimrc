set encoding=utf-8
set showcmd
set autochdir
filetype plugin indent on       "" load file type plugins + indendation

"" Whitespace
set relativenumber	          	"" for easier jumping 
set nowrap	            	     	"" don't wrap lines
set tabstop=2	shiftwidth=2    	"" 2 space tabs
set expandtab			              "" use spaces, not tabs
set backspace=indent,eol,start	"" backspace through everything in insert mode
set cindent                     "" set c-style indentation

"" Searching
set hlsearch                    "" highlight matches
set incsearch                   "" incremental searching
set ignorecase                  "" searches are case insensitive
set smartcase                   "" unless they contain at least one capital letter

"" for easier escaping
inoremap jk <C-[>
nnoremap <C-J> <C-W><C-J>
"" for easier split nav
nnoremap <C-K> <C-W><C-K>
nnoremap <C-L> <C-W><C-L>
nnoremap <C-H> <C-W><C-H>

autocmd BufRead,BufNewFile *.launch setfiletype roslaunch
