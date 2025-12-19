import React from 'react';
import OriginalMDXComponents from '@theme-original/MDXComponents';
import TypographyWrapper from '../components/TypographySystem';

// Custom heading components that use the typography system
const H1 = (props) => (
  <TypographyWrapper variant="heading" pageType="textbook" element="h1" {...props}>
    {props.children}
  </TypographyWrapper>
);

const H2 = (props) => (
  <TypographyWrapper variant="heading" pageType="textbook" element="h2" {...props}>
    {props.children}
  </TypographyWrapper>
);

const H3 = (props) => (
  <TypographyWrapper variant="heading" pageType="textbook" element="h3" {...props}>
    {props.children}
  </TypographyWrapper>
);

const H4 = (props) => (
  <TypographyWrapper variant="heading" pageType="textbook" element="h4" {...props}>
    {props.children}
  </TypographyWrapper>
);

const H5 = (props) => (
  <TypographyWrapper variant="heading" pageType="textbook" element="h5" {...props}>
    {props.children}
  </TypographyWrapper>
);

const H6 = (props) => (
  <TypographyWrapper variant="heading" pageType="textbook" element="h6" {...props}>
    {props.children}
  </TypographyWrapper>
);

const MDXComponents = {
  ...OriginalMDXComponents,
  h1: H1,
  h2: H2,
  h3: H3,
  h4: H4,
  h5: H5,
  h6: H6,
};

export default MDXComponents;