/**
 * Performance monitoring utilities for the landing page
 * Measures Core Web Vitals and other performance metrics
 */

// Check if we're in the browser environment
const isBrowser = typeof window !== 'undefined';

/**
 * Measure and log Core Web Vitals
 */
export const measureCoreWebVitals = () => {
  if (!isBrowser || !('PerformanceObserver' in window)) {
    console.warn('Performance Observer not supported in this browser');
    return;
  }

  // Measure Largest Contentful Paint (LCP)
  new PerformanceObserver((entryList) => {
    const entries = entryList.getEntries();
    const lastEntry = entries[entries.length - 1];
    console.log('LCP:', lastEntry.startTime);

    // Check if LCP meets the good threshold (<2.5s)
    if (lastEntry.startTime <= 2500) {
      console.log('✅ LCP Good:', lastEntry.startTime, 'ms');
    } else {
      console.log('⚠️ LCP Needs Improvement:', lastEntry.startTime, 'ms');
    }
  }).observe({ entryTypes: ['largest-contentful-paint'] });

  // Measure First Input Delay (FID) / Interaction to Next Paint (INP)
  new PerformanceObserver((entryList) => {
    const entries = entryList.getEntries();
    entries.forEach((entry) => {
      console.log('FID candidate:', entry.processingStart - entry.startTime);

      // Check if FID meets the good threshold (<100ms)
      if (entry.processingStart - entry.startTime <= 100) {
        console.log('✅ FID Good:', entry.processingStart - entry.startTime, 'ms');
      } else {
        console.log('⚠️ FID Needs Improvement:', entry.processingStart - entry.startTime, 'ms');
      }
    });
  }).observe({ entryTypes: ['first-input'] });

  // Measure Cumulative Layout Shift (CLS)
  let clsValue = 0;
  new PerformanceObserver((entryList) => {
    const entries = entryList.getEntries();
    for (const entry of entries) {
      if (!entry.hadRecentInput) {
        clsValue += entry.value;
      }
    }
    console.log('Current CLS:', clsValue);

    // Check if CLS meets the good threshold (<0.1)
    if (clsValue <= 0.1) {
      console.log('✅ CLS Good:', clsValue);
    } else {
      console.log('⚠️ CLS Needs Improvement:', clsValue);
    }
  }).observe({ entryTypes: ['layout-shift'] });
};

/**
 * Measure page load performance
 */
export const measurePageLoad = () => {
  if (!isBrowser) return;

  window.addEventListener('load', () => {
    // Get timing information
    const timing = performance.timing;
    const loadTime = timing.loadEventEnd - timing.navigationStart;
    const domReady = timing.domContentLoadedEventEnd - timing.navigationStart;
    const firstPaint = timing.responseEnd - timing.navigationStart;

    console.log('Page Load Metrics:');
    console.log('- DOM Ready:', domReady, 'ms');
    console.log('- Load Complete:', loadTime, 'ms');
    console.log('- First Paint:', firstPaint, 'ms');

    // Check if load time is under 3 seconds
    if (loadTime <= 3000) {
      console.log('✅ Page Load Time Good:', loadTime, 'ms');
    } else {
      console.log('⚠️ Page Load Time Needs Improvement:', loadTime, 'ms');
    }
  });
};

/**
 * Initialize performance monitoring
 */
export const initPerformanceMonitoring = () => {
  measureCoreWebVitals();
  measurePageLoad();
};

// Auto-initialize when the module is loaded
if (isBrowser) {
  // Wait for the page to be fully loaded
  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', initPerformanceMonitoring);
  } else {
    initPerformanceMonitoring();
  }
};

export default {
  measureCoreWebVitals,
  measurePageLoad,
  initPerformanceMonitoring
};