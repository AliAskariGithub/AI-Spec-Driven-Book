import React from 'react';
import clsx from 'clsx';

const SidebarEnhancement = ({ children, className = '', ...props }) => {
  return (
    <aside className={clsx('sidebar-enhancement', className)} {...props}>
      {children}
    </aside>
  );
};

export default SidebarEnhancement;